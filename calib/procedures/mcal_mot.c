/*
 * Phoenix-Pilot
 *
 * calib/procedures - magnetometer calibration with engines interference
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

#include <libsensors.h>

#include <sensc.h>
#include <mctl.h>
#include <vec.h>
#include <matrix.h>

#include "calls.h"


#define AVG_SAMPLES 100
#define AVG_WAIT 1000 * 20
#define CALIB_POINTS 10

/* FIXME: this should be handled inside, or taken from mtcl */
#define NUM_OF_MOTORS 4


/* 
* Quadratic Least Square Method. Solving matrix formula X = f(A, B) for 
* obtaining best fitting quadratic curve to measurement points.
* Constant measurement error assumed accros all samples.
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
* TODO: chceck for data corectness before inverting matrix (det(A) != 0)
*/
static int cal_qlsmFit(float *x, float *y, unsigned int n, float *a, float *b, float *c)
{
	int i;
	float SX4, SX3, SX2, SX, SXXY, SXY, SY;
	float bufA[9], bufB[3], bufX[3];
	matrix_t A = {.cols = 3, .rows = 3, .transposed = 0, .data = bufA};
	matrix_t B = {.cols = 1, .rows = 3, .transposed = 0, .data = bufB};
	matrix_t X = {.cols = 1, .rows = 3, .transposed = 0, .data = bufX};
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

	return 0;
}


static void cal_magAvgGet(vec_t *out, unsigned int samples)
{
	unsigned int i;
	sensor_event_t accelEvt, gyroEvt, magEvt;

	out->x = out->y = out->z = 0;
	for (i = 0; i < samples; i++) {
		sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		out->x += magEvt.mag.magX;
		out->y += magEvt.mag.magY;
		out->z += magEvt.mag.magZ;
		usleep(AVG_WAIT);
	}
	vec_times(out, 1./samples);
}

/* main function of this calibration module. Arms engines, runs a test and saves the line fitting */
int cal_mMotCalib(void)
{
	vec_t magBase, magCurr, magDiff;
	float thrtl = 0, thrtlStep;
	float x[CALIB_POINTS], y[3][CALIB_POINTS];
	unsigned int m, pts;
	float dummyVal;

	/* arm motors in safe mode. Warnings displayed by mctl_arm() */
	if (mctl_arm(1) < 0) {
		return -1;
	}

	/* get base magnetometer reading */
	cal_magAvgGet(&magBase, AVG_SAMPLES);

	/* skips full throttle, does not matter much */
	thrtlStep = 0.7 / CALIB_POINTS;
	for (pts = 0; pts < CALIB_POINTS; pts++) {
		for (m = 0; m < NUM_OF_MOTORS; m++) {
			if (mctl_thrtlSet(m, thrtl, tempoHigh) < 0) {
				mctl_disarm();
				return -1;
			}
		}

		cal_magAvgGet(&magCurr, AVG_SAMPLES);
		vec_dif(&magBase, &magCurr, &magDiff);

		x[pts] = thrtl;
		y[0][pts] = magDiff.x;
		y[1][pts] = magDiff.y;
		y[2][pts] = magDiff.z;

		thrtl += thrtlStep;
	}

	/* turn engines off */
	for (m = 0; m < NUM_OF_MOTORS; m++) {
		if (mctl_thrtlSet(m, 0, tempoInst) < 0) {
			mctl_disarm();
			return -1;
		}
	}
	mctl_disarm();

	/*
	* Fitting parabola with Quadratic Least Square Method Fitting function.
	* Discarding calculated constant as it should be only an effect of measurements inacuracy and should be low.
	*/
	cal_qlsmFit(x, y[0], CALIB_POINTS, &calibs_common.mMot.ax, &calibs_common.mMot.bx, &dummyVal);
	cal_qlsmFit(x, y[1], CALIB_POINTS, &calibs_common.mMot.ay, &calibs_common.mMot.by, &dummyVal);
	cal_qlsmFit(x, y[2], CALIB_POINTS, &calibs_common.mMot.az, &calibs_common.mMot.bz, &dummyVal);

	return 0;
}
