/*
 * Phoenix-Pilot
 *
 * Ellipsoidal calibration common functions
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <libsensors.h>

#include <sensc.h>
#include <vec.h>
#include <matrix.h>


#define ELLCAL_TAG        "ellcal"
#define LMA_JACOBIAN_STEP 0.0001


/*
* Target function has a form  of: || S(X - H) || = 1 where:
*
* X -> 3x1 matrix of (x,y,z) uncalibrated measurement
* S -> 3x3 ellipsoid deformation matrix of form:
*    | p0, p1, p2 |
*    | p3, p4, p5 |
*    | p6, p7, p8 |
* H -> 3x1 ellipsoid center offset matrix of form: (p9, p10, p11)^T
*
* We want this function to transform data from some ellipsoid into a unit radius sphere.
* This function does not check any conic function equalities for ellipsoid.
*
* Works well if data is placed around (0,0,0) +- (1,1,1) and the biggest ellipsoid semiaxis is of length ~1.
*/
int ellcal_lmaResiduum(const matrix_t *P, const matrix_t *V, float *res, bool log)
{
	const matrix_t S = { .data = &P->data[0], .rows = 3, .cols = 3, .transposed = 0 };
	const matrix_t H = { .data = &P->data[9], .rows = 3, .cols = 1, .transposed = 0 };

	int i;
	float r = 0;
	float dataX[3], dataTmpX[3];
	matrix_t X = { .data = dataX, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t tmpX = { .data = dataTmpX, .rows = 3, .cols = 1, .transposed = 0 };

	for (i = 0; i < 3; i++) {
		MATRIX_DATA(&X, i, 0) = MATRIX_DATA(V, 0, i); /* Copy V into X matrix */
	}

	matrix_sub(&X, &H, NULL);   /* apply offset */
	matrix_prod(&S, &X, &tmpX); /* apply ellipsoid deformation matrix */

	/* calculate length of produced vector */
	for (i = 0; i < 3; i++) {
		r += MATRIX_DATA(&tmpX, i, 0) * MATRIX_DATA(&tmpX, i, 0);
	}

	/* Length difference between transformed vector and a unit sphere radius is the residuum */
	*res = (float)(sqrt(r) - 1);

	return 0;
}


/*
* Jacobian is calculated numerically by calculating differences of residuum after step in parameters.
* Steps are taken for each parameter at a time.
*/
int ellcal_lmaJacobian(const matrix_t *P, const matrix_t *V, matrix_t *J, bool log)
{
	float dataPtmp[12];
	matrix_t Ptmp = { .data = dataPtmp, .rows = 1, .cols = 12, .transposed = 0 };
	float baseRes, newRes;
	int i;

	ellcal_lmaResiduum(P, V, &baseRes, false);

	/* Numeric jacobian solving for each parameter */
	for (i = 0; i < 12; i++) {
		memcpy(Ptmp.data, P->data, 12 * sizeof(float));

		/* Increment, get new residuum and calculate the slope of change */
		MATRIX_DATA(&Ptmp, 0, i) += LMA_JACOBIAN_STEP;
		ellcal_lmaResiduum(&Ptmp, V, &newRes, false);
		MATRIX_DATA(J, 0, i) = (newRes - baseRes) / LMA_JACOBIAN_STEP;
	}

	return 0;
}


/* Initial guess is unit radius sphere at (0,0,0) */
void ellcal_lmaGuess(matrix_t *P)
{
	/* ellipsoid deformation guess */
	MATRIX_DATA(P, 0, 0) = 1;
	MATRIX_DATA(P, 0, 1) = 0;
	MATRIX_DATA(P, 0, 2) = 0;
	MATRIX_DATA(P, 0, 3) = 0;
	MATRIX_DATA(P, 0, 4) = 1;
	MATRIX_DATA(P, 0, 5) = 0;
	MATRIX_DATA(P, 0, 6) = 0;
	MATRIX_DATA(P, 0, 7) = 0;
	MATRIX_DATA(P, 0, 8) = 1;

	/* ellipsoid offset guess */
	MATRIX_DATA(P, 0, 9) = 0;
	MATRIX_DATA(P, 0, 10) = 0;
	MATRIX_DATA(P, 0, 11) = 0;
}


void ellcal_lma2matrices(const matrix_t *lmaParams, matrix_t *S, matrix_t *H)
{
	MATRIX_DATA(S, 0, 0) = MATRIX_DATA(lmaParams, 0, 0);
	MATRIX_DATA(S, 0, 1) = MATRIX_DATA(lmaParams, 0, 1);
	MATRIX_DATA(S, 0, 2) = MATRIX_DATA(lmaParams, 0, 2);
	MATRIX_DATA(S, 1, 0) = MATRIX_DATA(lmaParams, 0, 3);
	MATRIX_DATA(S, 1, 1) = MATRIX_DATA(lmaParams, 0, 4);
	MATRIX_DATA(S, 1, 2) = MATRIX_DATA(lmaParams, 0, 5);
	MATRIX_DATA(S, 2, 0) = MATRIX_DATA(lmaParams, 0, 6);
	MATRIX_DATA(S, 2, 1) = MATRIX_DATA(lmaParams, 0, 7);
	MATRIX_DATA(S, 2, 2) = MATRIX_DATA(lmaParams, 0, 8);

	MATRIX_DATA(H, 0, 0) = MATRIX_DATA(lmaParams, 0, 9);
	MATRIX_DATA(H, 1, 0) = MATRIX_DATA(lmaParams, 0, 10);
	MATRIX_DATA(H, 2, 0) = MATRIX_DATA(lmaParams, 0, 11);
}


static int ellcal_gyroBiasGet(vec_t *bias)
{
	sensor_event_t accelEvt, gyroEvt, magEvt;
	vec_t gyroBias;
	int i;

	printf("Do not rotate the device for 1s after pressing [Enter]\n");
	printf("Press [Enter] to continue...");
	fflush(stdout);
	getchar();

	/* Calculating gyroscope bias */
	for (i = 0; i < 1000; i++) {
		if (sensc_imuGet(&accelEvt, &gyroEvt, &magEvt) < 0) {
			fprintf(stderr, "%s: sensc_imuGet() fail\n", ELLCAL_TAG);
			return -1;
		}
		gyroBias.x += gyroEvt.gyro.gyroX / 1000.f;
		gyroBias.y += gyroEvt.gyro.gyroY / 1000.f;
		gyroBias.z += gyroEvt.gyro.gyroZ / 1000.f;
		usleep(1000);
	}
	vec_times(&gyroBias, 0.001);

	*bias = gyroBias;

	return 0;
}

/*
* Taking samples each time we rotate more than `deltaAngle` radians.
* Total of `nSamples` is saved to `samplesBuf`
* Angle integration implemented to assure correct angles of rotation.
*
* Assumed constant gyroscope bias and neglecting gyroscope drift.
*/
int ellcal_rotDataGet(vec_t *samplesBuf, size_t nSamples, float deltaAngle, int sensorType)
{
	sensor_event_t accelEvt, gyroEvt, magEvt;
	vec_t angle, gyroBias = { 0 };
	time_t lastT, currT;
	float deltaT;
	int i, nPrint;

	if (ellcal_gyroBiasGet(&gyroBias) < 0) {
		return -1;
	}

	printf("Rotate the device until all samples are taken\n");
	printf("Press [Enter] to begin sampling...");
	fflush(stdout);
	getchar();
	printf("Rotate...\n");

	/*
	* Taking samples each time we rotate more than DANGLE_MIN radians.
	* Angle integration implemented to assure correct angles of rotation.
	*
	* Assumed constant gyroscope bias and neglecting gyroscope drift.
	*/
	gettime(&lastT, NULL);
	currT = lastT;
	i = 0;
	nPrint = 0;
	while (i < nSamples) {
		usleep(1000);
		gettime(&currT, NULL);

		if (sensc_imuGet(&accelEvt, &gyroEvt, &magEvt) < 0) {
			fprintf(stderr, "%s: sensc_imuGet() fail\n", ELLCAL_TAG);
		}

		deltaT = (float)(currT - lastT) / 1000000.f;

		/* Conversion from mrad/s -> rad/s */
		angle.x += (gyroEvt.gyro.gyroX / 1000.f - gyroBias.x) * deltaT;
		angle.y += (gyroEvt.gyro.gyroY / 1000.f - gyroBias.y) * deltaT;
		angle.z += (gyroEvt.gyro.gyroZ / 1000.f - gyroBias.z) * deltaT;

		if (vec_len(&angle) > deltaAngle) {
			switch (sensorType) {
				case SENSOR_TYPE_MAG:
					samplesBuf[i].x = magEvt.mag.magX;
					samplesBuf[i].y = magEvt.mag.magY;
					samplesBuf[i].z = magEvt.mag.magZ;
					break;

				case SENSOR_TYPE_ACCEL:
					samplesBuf[i].x = accelEvt.accels.accelX;
					samplesBuf[i].y = accelEvt.accels.accelY;
					samplesBuf[i].z = accelEvt.accels.accelZ;
					break;

				default:
					fprintf(stderr, "%s: unknown sensortype: %d\n", ELLCAL_TAG, sensorType);
					return -1;
			}

			if (i % (nSamples / 100) == (nSamples / 100) - 1) {
				printf("%*c\r", nPrint, ' ');
				nPrint = printf("%s: Taken samples: %u/%u", ELLCAL_TAG, i, nSamples);
				fflush(stdout);
			}

			angle = (vec_t) { .x = 0, .y = 0, .z = 0 };
			i++;
		}

		lastT = currT;
	}
	printf("\n");

	return 0;
}
