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
#include "sensc.h"


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
	calib_data_t gyrorth;

	int corrInitFlags;

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

	if ((corr_common.corrInitFlags & CORR_ENBL_MAGMOT) != 0) {
		for (i = 0; i < NUM_OF_MOTORS; i++) {
			fclose(corr_common.pwmFiles[i]);
		}
		calib_free(&corr_common.magmot);
	}

	if ((corr_common.corrInitFlags & CORR_ENBL_MAGIRON) != 0) {
		calib_free(&corr_common.magiron);
	}

	if ((corr_common.corrInitFlags & CORR_ENBL_ACCORTH) != 0) {
		calib_free(&corr_common.accorth);
	}

	if ((corr_common.corrInitFlags & CORR_ENBL_TEMPIMU) != 0) {
		calib_free(&corr_common.tempimu);
	}
}


int corr_init(int initFlags)
{
	int i;
	bool err = false;

	corr_common.corrInitFlags = CORR_ENBL_NONE;

	/* MAGIRON initialization */
	if ((initFlags & CORR_ENBL_MAGIRON) != 0) {
		if (calib_dataInit(CALIB_PATH, typeMagiron, &corr_common.magiron) == 0) {
			corr_common.corrInitFlags |= CORR_ENBL_MAGIRON;
		}
		else {
			fprintf(stderr, "corr: %s init failed\n", MAGIRON_TAG);
			err = true;
		}
	}

	/* MAGMOT initialization */
	if ((initFlags & CORR_ENBL_MAGMOT) != 0 && !err) {
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

		if (calib_dataInit(CALIB_PATH, typeMagmot, &corr_common.magmot) == 0) {
			corr_common.corrInitFlags |= CORR_ENBL_MAGMOT;
		}
		else {
			fprintf(stderr, "corr: %s init failed\n", MAGMOT_TAG);
			err = true;
		}
	}

	/* ACCORTH initialization */
	if ((initFlags & CORR_ENBL_ACCORTH) != 0 && !err) {
		if (calib_dataInit(CALIB_PATH, typeAccorth, &corr_common.accorth) == 0) {
			corr_common.corrInitFlags |= CORR_ENBL_ACCORTH;
		}
		else {
			fprintf(stderr, "corr: %s init failed\n", ACCORTH_TAG);
			err = true;
		}
	}

	if ((initFlags & CORR_ENBL_GYRORTH) != 0 && !err) {
		if (calib_dataInit(CALIB_PATH, typeGyrorth, &corr_common.gyrorth) == 0) {
			corr_common.corrInitFlags |= CORR_ENBL_GYRORTH;
		}
		else {
			fprintf(stderr, "corr: %s init failed\n", GYRORTH_TAG);
			err = true;
		}
	}

	/* TEMPIMU initialization */
	if ((initFlags & CORR_ENBL_TEMPIMU) != 0 && !err) {
		if (calib_dataInit(CALIB_PATH, typeTempimu, &corr_common.tempimu) != 0) {
			fprintf(stderr, "corr: %s init failed\n", TEMPIMU_TAG);
			err = true;
		}
		else {
			corr_common.corrInitFlags |= CORR_ENBL_TEMPIMU;
		}
	}

	/* error checking */
	if (err) {
		if ((corr_common.corrInitFlags & CORR_ENBL_MAGMOT) != 0) {
			for (i = 0; i < NUM_OF_MOTORS; i++) {
				fclose(corr_common.pwmFiles[i]);
			}
			calib_free(&corr_common.magmot);
		}

		if ((corr_common.corrInitFlags & CORR_ENBL_MAGIRON) != 0) {
			calib_free(&corr_common.magiron);
		}

		if ((corr_common.corrInitFlags & CORR_ENBL_ACCORTH) != 0) {
			calib_free(&corr_common.accorth);
		}

		if ((corr_common.corrInitFlags & CORR_ENBL_TEMPIMU) != 0) {
			calib_free(&corr_common.tempimu);
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


static void corr_magmot(sensor_event_t *magEvt)
{
	static time_t lastRecal = 0;     /* last magmot recalculation time */
	static vec_t magmotCorr = { 0 }; /* magmot correction */

	/* decide on refreshing calibration arguments */
	if (magEvt->timestamp - lastRecal > MAGMOT_MAXPERIOD) {
		lastRecal = magEvt->timestamp;

		corr_magmotRecalc(&magmotCorr);
	}

	/* Apply magmot correction */
	magEvt->mag.magX += magmotCorr.x;
	magEvt->mag.magY += magmotCorr.y;
	magEvt->mag.magZ += magmotCorr.z;
}


static void corr_magiron(sensor_event_t *magEvt)
{
	float corrBuff[3], measBuf[3] = { magEvt->mag.magX, magEvt->mag.magY, magEvt->mag.magZ };
	matrix_t meas = { .data = measBuf, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t corr = { .data = corrBuff, .rows = 3, .cols = 1, .transposed = 0 };

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
}


static void corr_accrotVecSwap(vec_t *v)
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


static void corr_accrot(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
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


static void corr_accorth(sensor_event_t *accelEvt)
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


static void corr_tempimu(sensor_event_t *accelEvt, sensor_event_t *gyroEvt)
{
	float diff;

	if (accelEvt->accels.temp != 0) {
		diff = ((float)accelEvt->accels.temp) / 1000 - corr_common.tempimu.params.tempimu.refTemp;
		accelEvt->accels.accelX -= diff * corr_common.tempimu.params.tempimu.alfaAcc[0];
		accelEvt->accels.accelY -= diff * corr_common.tempimu.params.tempimu.alfaAcc[1];
		accelEvt->accels.accelZ -= diff * corr_common.tempimu.params.tempimu.alfaAcc[2];
	}

	/* Only correcting direct measurement, as dAngle is hard to correct without timestamps */
	if (gyroEvt->gyro.temp != 0) {
		diff = ((float)gyroEvt->gyro.temp) / 1000 - corr_common.tempimu.params.tempimu.refTemp;
		gyroEvt->gyro.gyroX -= diff * corr_common.tempimu.params.tempimu.alfaGyr[0];
		gyroEvt->gyro.gyroY -= diff * corr_common.tempimu.params.tempimu.alfaGyr[1];
		gyroEvt->gyro.gyroZ -= diff * corr_common.tempimu.params.tempimu.alfaGyr[2];
	}
}


static void corr_gyrorth(sensor_event_t *gyroEvt)
{
	float dataFinal[3];
	float dataTmp[3] = { gyroEvt->gyro.gyroX, gyroEvt->gyro.gyroY, gyroEvt->gyro.gyroZ };
	matrix_t tmp = { .data = dataTmp, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t final = { .data = dataFinal, .rows = 3, .cols = 1, .transposed = 0 };

	matrix_sub(&tmp, &corr_common.gyrorth.params.gyrorth.offset, NULL);
	matrix_prod(&corr_common.gyrorth.params.gyrorth.ortho, &tmp, &final);

	gyroEvt->gyro.gyroX = MATRIX_DATA(&final, 0, 0);
	gyroEvt->gyro.gyroY = MATRIX_DATA(&final, 1, 0);
	gyroEvt->gyro.gyroZ = MATRIX_DATA(&final, 2, 0);
}


void corr_imu(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
{
	/* Magnetometer corrections*/
	if ((corr_common.corrInitFlags & CORR_ENBL_MAGIRON) != 0) {
		corr_magiron(magEvt);
	}
	if ((corr_common.corrInitFlags & CORR_ENBL_MAGMOT) != 0) {
		corr_magmot(magEvt);
	}

	/* Accelerometer corrections */
	if ((corr_common.corrInitFlags & CORR_ENBL_TEMPIMU) != 0) {
		corr_tempimu(accelEvt, gyroEvt);
	}
	if ((corr_common.corrInitFlags & CORR_ENBL_ACCORTH) != 0) {
		/* TODO: split accrot from accorth */
		corr_accorth(accelEvt);
		corr_accrot(accelEvt, gyroEvt, magEvt);
	}
	if ((corr_common.corrInitFlags & CORR_ENBL_GYRORTH) != 0) {
		corr_gyrorth(gyroEvt);
	}
}
