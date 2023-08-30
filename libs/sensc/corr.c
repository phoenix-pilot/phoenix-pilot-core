/*
 * Phoenix-Pilot
 *
 * sensorhub client correction functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include <board_config.h>
#include <libsensors.h>
#include <calib.h>
#include <vec.h>
#include <matrix.h>

#include "corr.h"


#define NUM_OF_MOTORS 4

#define MAGMOT_MAXPERIOD 80000  /* 80ms */
#define PWM_PRESCALER    100000 /* according to mctl */


static const char *motorFiles[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};

struct {
	calib_data_t magmot;
	calib_data_t magiron;
	calib_data_t accorth;
	calib_data_t tempimu;

	/* for magmot correction */
	FILE *pwmFiles[NUM_OF_MOTORS];
} corr_common;


static inline void corr_motorImpact(vec_t *result, int motor, float throttle)
{
	unsigned int param;
	vec_t axisImpact[3];
	vec_t impact = { 0 };

	/*
	* Write parameters into vectors.
	* Each 'axisImpact' stores different quadratic formula parameters so later they can be used as 3d dimensional parameters
	*/
	for (param = 0; param < 3; param++) {
		axisImpact[param].x = corr_common.magmot.params.magmot.motorEq[motor][0][param];
		axisImpact[param].y = corr_common.magmot.params.magmot.motorEq[motor][1][param];
		axisImpact[param].z = corr_common.magmot.params.magmot.motorEq[motor][2][param];
	}

	/*
	* Perform (y = - ax^2 - bx - c) where:
	*  - x is throttle (scalar),
	*  - a/b/c are parameters (vectors),
	*  - y is impact of current motor on magnetometer reading (vector)
	*/
	vec_times(&axisImpact[0], throttle * throttle);
	vec_times(&axisImpact[1], throttle);

	vec_add(&impact, &axisImpact[0]);
	vec_add(&impact, &axisImpact[1]);
	vec_add(&impact, &axisImpact[2]);

	*result = impact;
}


void corr_done(void)
{
	int i;

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		fclose(corr_common.pwmFiles[i]);
	}

	calib_free(&corr_common.magiron);
	calib_free(&corr_common.magmot);
}


int corr_init(void)
{
	int magironRet, magmotRet, accorthRet, tempimuRet;
	int i;
	bool err = false;

	/* open pwm files for magmot correction */
	for (i = 0; i < NUM_OF_MOTORS; i++) {
		corr_common.pwmFiles[i] = fopen(motorFiles[i], "r");
		if (corr_common.pwmFiles[i] == NULL) {
			err = true;
			fprintf(stderr, "corr: cannot access %s pwm file\n", motorFiles[i]);
			break;
		}
	}

	if (err) {
		i--;
		while (i >= 0) {
			fclose(corr_common.pwmFiles[i]);
		}
		fprintf(stderr, "corr: failed to open motor files\n");
		return -1;
	}

	magironRet = calib_readFile(CALIB_PATH, typeMagiron, &corr_common.magiron);
	magmotRet = calib_readFile(CALIB_PATH, typeMagmot, &corr_common.magmot);
	accorthRet = calib_readFile(CALIB_PATH, typeAccorth, &corr_common.accorth);
	tempimuRet = calib_readFile(CALIB_PATH, typeTempimu, &corr_common.tempimu);

	/* error checking */
	if (magironRet != 0 || magmotRet != 0 || accorthRet != 0 || tempimuRet != 0) {

		(magmotRet == 0) ? calib_free(&corr_common.magiron) : fprintf(stderr, "corr: magmot init failed\n");
		(magironRet == 0) ? calib_free(&corr_common.magiron) : fprintf(stderr, "corr: magiron init failed\n");
		(accorthRet == 0) ? calib_free(&corr_common.accorth) : fprintf(stderr, "corr: accorth init failed\n");
		(tempimuRet == 0) ? calib_free(&corr_common.tempimu) : fprintf(stderr, "corr: tempimu init failed\n");

		for (i = 0; i < NUM_OF_MOTORS; i++) {
			fclose(corr_common.pwmFiles[i]);
		}
		return -1;
	}

	return 0;
}


static int corr_magmotRecalc(vec_t *correction)
{
	long throttles[NUM_OF_MOTORS];
	char buff[16];
	int motor;
	float throttle;
	vec_t impact, impactSum = { 0 };

	/* Read current throttles from pwmFiles */
	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		rewind(corr_common.pwmFiles[motor]);
		if (fread(buff, sizeof(char), sizeof(buff), corr_common.pwmFiles[motor]) < sizeof(buff)) {
			return -1;
		}

		throttles[motor] = strtod(buff, NULL);

		/* strtod fail also fails this check */
		throttles[motor] -= PWM_PRESCALER;
		if (throttles[motor] > PWM_PRESCALER || throttles[motor] < 0) {
			return -1;
		}
	}

	/* Calculating corrections motor-wise */
	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		throttle = (float)throttles[motor] / PWM_PRESCALER;

		corr_motorImpact(&impact, motor, throttle);

		/*
		* Calibration is done on throttles between (cutoff, 1).
		* Artifacts may appear below cutoff. Scaling them down with throttle value.
		*/
		if (throttle < MAGMOT_CUTOFF_THROTTLE) {
			vec_times(&impact, throttle / MAGMOT_CUTOFF_THROTTLE);
		}

		vec_add(&impactSum, &impact);
	}

	*correction = impactSum;

	return 0;
}


void corr_mag(sensor_event_t *magEvt)
{
	static time_t lastRecal = 0;     /* last magmot recalculation time */
	static vec_t magmotCorr = { 0 }; /* magmot correction */

	float corrBuff[3], measBuf[3] = { magEvt->mag.magX, magEvt->mag.magY, magEvt->mag.magZ };
	matrix_t meas = { .data = measBuf, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t corr = { .data = corrBuff, .rows = 3, .cols = 1, .transposed = 0 };

	/* decide on refreshing calibration arguments */
	if (magEvt->timestamp - lastRecal > MAGMOT_MAXPERIOD) {
		lastRecal = magEvt->timestamp;

		corr_magmotRecalc(&magmotCorr);
	}

	/*
	* Corrections MUST be applied in order:
	* 1) magiron
	* 2) magmot
	*/

	/*
	* Apply magiron correction
	* 1) Subtract the hardiron error
	* 2) Compensate softiron deformations by softcaal matrix with measurement
	*/
	matrix_sub(&meas, &corr_common.magiron.params.magiron.hardCal, NULL);
	matrix_prod(&corr_common.magiron.params.magiron.softCal, &meas, &corr);

	magEvt->mag.magX = *matrix_at(&corr, 0, 0);
	magEvt->mag.magY = *matrix_at(&corr, 1, 0);
	magEvt->mag.magZ = *matrix_at(&corr, 2, 0);

	/* Apply magmot correction */
	magEvt->mag.magX += magmotCorr.x;
	magEvt->mag.magY += magmotCorr.y;
	magEvt->mag.magZ += magmotCorr.z;
}


void corr_accrotVecSwap(vec_t *v)
{
	switch (corr_common.accorth.params.accorth.swapOrder) {
		case accSwapXZY:
			*v = (vec_t) { .x = v->x, .y = v->z, .z = v->y };
			break;

		case accSwapYXZ:
			*v = (vec_t) { .x = v->y, .y = v->x, .z = v->z };
			break;

		case accSwapYZX:
			*v = (vec_t) { .x = v->y, .y = v->z, .z = v->x };
			break;

		case accSwapZXY:
			*v = (vec_t) { .x = v->z, .y = v->x, .z = v->y };
			break;

		case accSwapZYX:
			*v = (vec_t) { .x = v->z, .y = v->y, .z = v->x };
			break;

		case accSwapXYZ:
		default:
			/* no swap */
			break;
	}

	if (corr_common.accorth.params.accorth.axisInv[0] == 1) {
		v->x = -v->x;
	}

	if (corr_common.accorth.params.accorth.axisInv[1] == 1) {
		v->y = -v->y;
	}

	if (corr_common.accorth.params.accorth.axisInv[2] == 1) {
		v->z = -v->z;
	}
}


void corr_accrot(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
{
	vec_t accel = { .x = accelEvt->accels.accelX, .y = accelEvt->accels.accelY, .z = accelEvt->accels.accelZ };
	vec_t gyro = { .x = gyroEvt->gyro.gyroX, .y = gyroEvt->gyro.gyroY, .z = gyroEvt->gyro.gyroZ };
	vec_t mag = { .x = magEvt->mag.magX, .y = magEvt->mag.magY, .z = magEvt->mag.magZ };

	corr_accrotVecSwap(&accel);
	corr_accrotVecSwap(&gyro);
	corr_accrotVecSwap(&mag);

	quat_vecRot(&accel, &corr_common.accorth.params.accorth.frameQ);
	quat_vecRot(&gyro, &corr_common.accorth.params.accorth.frameQ);
	quat_vecRot(&mag, &corr_common.accorth.params.accorth.frameQ);

	accelEvt->accels.accelX = accel.x;
	accelEvt->accels.accelY = accel.y;
	accelEvt->accels.accelZ = accel.z;

	gyroEvt->gyro.gyroX = gyro.x;
	gyroEvt->gyro.gyroY = gyro.y;
	gyroEvt->gyro.gyroZ = gyro.z;

	magEvt->mag.magX = mag.x;
	magEvt->mag.magY = mag.y;
	magEvt->mag.magZ = mag.z;
}


void corr_accorth(sensor_event_t *accelEvt)
{
	float dataFinal[3];
	float dataTmp[3] = { accelEvt->accels.accelX, accelEvt->accels.accelY, accelEvt->accels.accelZ };
	matrix_t tmp = { .data = dataTmp, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t final = { .data = dataFinal, .rows = 3, .cols = 1, .transposed = 0 };

	matrix_sub(&tmp, &corr_common.accorth.params.accorth.offset, NULL);
	matrix_prod(&corr_common.accorth.params.accorth.ortho, &tmp, &final);

	accelEvt->accels.accelX = MATRIX_DATA(&final, 0, 0);
	accelEvt->accels.accelY = MATRIX_DATA(&final, 1, 0);
	accelEvt->accels.accelZ = MATRIX_DATA(&final, 2, 0);
}
