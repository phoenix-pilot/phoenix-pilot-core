/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against motor interference
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <board_config.h>
#include <libsensors.h>

#include <sensc.h>
#include <mctl.h>
#include <vec.h>
#include <matrix.h>
#include <calib.h>

#include "calibtool.h"


#define AVG_SAMPLES  100
#define AVG_WAIT     (1000 * 10)
#define CALIB_POINTS 10

/* FIXME: this should be handled inside, or taken from mtcl */
#define NUM_OF_MOTORS 4

static const char *motorFiles[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};

struct {
	calib_data_t data;
} magmot_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *magmot_dataGet(void)
{
	return &magmot_common.data;
}


/*
* Quadratic Least Square Method. Solving matrix formula X = f(A, B) for
* obtaining best fitting quadratic curve to measurement points.
* Constant measurement error assumed across all samples.
*
* f(A, B) = A^(-1) * B
*
* X is a matrix of quadratic formula (y = ax^2 + bx + c) coefficients
* X = [a, b, c]^T
*
* A is matrix of coefficients obtained from measurement points as follows:
* A=| sum(x_i^4), sum(x_i^3), sum(x_i^2) |
*	| sum(x_i^3), sum(x_i^2), sum(x_i^1) |
*	| sum(x_i^2), sum(x_i^1),   sum(n)   |
*
* B is matrix of coefficients obtained from measurement points as follows:
* B = [ sum(x_i^2 * y_i), sum(x_i * y_i), sum(y_i) ]^T
*
* TODO: chceck for data correctness before inverting matrix (det(A) != 0)
*/
static void magmot_qlsmFit(float *x, float *y, unsigned int n, float *a, float *b, float *c)
{
	int i;
	float SX4, SX3, SX2, SX, SXXY, SXY, SY;
	float bufA[9], bufB[3], bufX[3];
	matrix_t A = { .cols = 3, .rows = 3, .transposed = 0, .data = bufA };
	matrix_t B = { .cols = 1, .rows = 3, .transposed = 0, .data = bufB };
	matrix_t X = { .cols = 1, .rows = 3, .transposed = 0, .data = bufX };
	float bufInv[2 * 9];

	matrix_zeroes(&A);
	matrix_zeroes(&B);
	matrix_zeroes(&X);

	SX4 = SX3 = SX2 = SX = 0;
	SXXY = SXY = SY = 0;

	for (i = 0; i < n; i++) {
		SX4 += x[i] * x[i] * x[i] * x[i];
		SX3 += x[i] * x[i] * x[i];
		SX2 += x[i] * x[i];
		SXY += x[i] * y[i];
		SX += x[i];
		SY += y[i];
		SXXY += x[i] * x[i] * y[i];
	}

	/* fill A matrix */
	*matrix_at(&A, 0, 0) = SX4;
	*matrix_at(&A, 1, 0) = *matrix_at(&A, 0, 1) = SX3;
	*matrix_at(&A, 2, 0) = *matrix_at(&A, 1, 1) = *matrix_at(&A, 0, 2) = SX2;
	*matrix_at(&A, 2, 1) = *matrix_at(&A, 1, 2) = SX;
	*matrix_at(&A, 2, 2) = n;

	/* fill B matrix */
	*matrix_at(&B, 0, 0) = SXXY;
	*matrix_at(&B, 1, 0) = SXY;
	*matrix_at(&B, 2, 0) = SY;

	/* calculate column matrix X from prepared matrices */
	matrix_inv(&A, &A, bufInv, 2 * 9);
	matrix_prod(&A, &B, &X);

	/* copy coefficients outside */
	*a = *matrix_at(&X, 0, 0);
	*b = *matrix_at(&X, 1, 0);
	*c = *matrix_at(&X, 2, 0);
}


/* Get average magnetometer reading over `n` samples */
static void magmot_magAvg(vec_t *out, unsigned int n)
{
	unsigned int i;
	sensor_event_t accelEvt, gyroEvt, magEvt;

	out->x = out->y = out->z = 0;
	for (i = 0; i < n; i++) {
		sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		out->x += magEvt.mag.magX;
		out->y += magEvt.mag.magY;
		out->z += magEvt.mag.magZ;
		usleep(AVG_WAIT);
	}
	vec_times(out, 1. / n);
}


/*
* Writes proper name for calibration parameter of:
* - 'paramId' equation parameter for...
* - 'axisId' magnetometer axis...
* - of 'motorId' motor interference impact in relation to its throttle value.
*
* 'buf' must be allocated and sizeof(buf) = 5.
*/
static void magmot_paramName(unsigned int motorId, unsigned int axisId, unsigned int paramId, char *buf)
{
	char xyz[3] = "xyz";
	char abc[3] = "abc";

	sprintf(buf, "m%i%c%c", motorId, xyz[axisId], abc[paramId]);
}


static int magmot_write(FILE *file)
{
	unsigned int motor, axis, param;
	char paramName[5];

	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		for (axis = 0; axis < 3; axis++) {
			for (param = 0; param < 3; param++) {
				magmot_paramName(motor, axis, param, paramName);
				fprintf(file, "%s %f\n", paramName, magmot_common.data.params.magmot.motorEq[motor][axis][param]);
			}
		}
	}
	return 0;
}


const char *magmot_help(void)
{
	return "Magnetometer vs engine interference calibration\n";
}


static int magmot_run(void)
{
	vec_t magBase, magCurr, magDiff;
	float thrtl = 0, thrtlStep, startThrtl = MAGMOT_CUTOFF_THROTTLE;
	float x[CALIB_POINTS], y[3][CALIB_POINTS];
	unsigned int m, pts;
	float (*motorEq)[3][3];

	/* arm motors in safe mode. Warnings displayed by mctl_arm() */
	if (mctl_arm(armMode_user) < 0) {
		return -ENXIO;
	}

	/* get base magnetometer reading */
	magmot_magAvg(&magBase, AVG_SAMPLES);

	thrtlStep = (1.0 - startThrtl) / CALIB_POINTS;
	for (m = 0; m < NUM_OF_MOTORS; m++) {
		thrtl = startThrtl;
		for (pts = 0; pts < CALIB_POINTS; pts++) {
			if (mctl_thrtlSet(m, thrtl, tempoHigh) < 0) {
				mctl_disarm();
				return -ENXIO;
			}

			magmot_magAvg(&magCurr, AVG_SAMPLES);
			vec_dif(&magBase, &magCurr, &magDiff);

			x[pts] = thrtl;
			y[0][pts] = magDiff.x;
			y[1][pts] = magDiff.y;
			y[2][pts] = magDiff.z;

			printf("%f %f %f %f\n", thrtl, magDiff.x, magDiff.y, magDiff.z);

			thrtl += thrtlStep;
		}

		if (mctl_thrtlSet(m, 0, tempoInst) < 0) {
			mctl_disarm();
			return -ENXIO;
		}
		usleep(1000 * 400); /* wait for engine to slow down */

		/* aliasing for better readability */
		motorEq = magmot_common.data.params.magmot.motorEq;

		/* fitting quadratic equation for magnetometer x-axis interference from m-th engine */
		magmot_qlsmFit(x, y[0], CALIB_POINTS, &motorEq[m][0][0], &motorEq[m][0][1], &motorEq[m][0][2]);
		/* fitting quadratic equation for magnetometer y-axis interference from m-th engine */
		magmot_qlsmFit(x, y[1], CALIB_POINTS, &motorEq[m][1][0], &motorEq[m][1][1], &motorEq[m][1][2]);
		/* fitting quadratic equation for magnetometer z-axis interference from m-th engine */
		magmot_qlsmFit(x, y[2], CALIB_POINTS, &motorEq[m][2][0], &motorEq[m][2][1], &motorEq[m][2][2]);
	}
	mctl_disarm();

	return EOK;
}


static int magmot_done(void)
{
	sensc_deinit();
	mctl_deinit();

	return EOK;
}


static int magmot_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, MAGMOT_CALIB_DEPENDENCY, SENSC_INIT_IMU) < 0) {
		return -ENXIO;
	}

	if (mctl_init(NUM_OF_MOTORS, motorFiles) < 0) {
		mctl_deinit();
		return -ENXIO;
	}

	return EOK;
}


__attribute__((constructor(102))) static void magmot_register(void)
{
	static calib_ops_t cal = {
		.name = MAGMOT_TAG,
		.init = magmot_init,
		.run = magmot_run,
		.done = magmot_done,
		.write = magmot_write,
		.help = magmot_help,
		.dataGet = magmot_dataGet
	};

	calib_register(&cal);

	magmot_common.data.type = typeMagmot;
}
