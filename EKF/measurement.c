/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * data acquisition logic/calibration
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/msg.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

float imu_raw[10];
vec_t meas_data[3];
float g_scaleerr_common = 1;

/* accelerometer calibration data */
float tax, tay, taz;
float acc_calib1[12] = {
	/* sphere offset */
	0.01488213, -0.01709187, 0.01382025,
	/* sphere deformation */
	1.00240603e+00, 2.64122891e-03, 4.00890692e-04,
	2.64122891e-03, 1.00256739e+00, 2.65286398e-03,
	4.00890692e-04, 2.65286398e-03, 9.95059154e-01
};
float acc_finecalib[12] = {
	-0.005796, -0.000178, -0.000178,
	0.985368, 0, 0,
	0, 0.982323, 0,
	0, 0, 0.985387
};

/* accelerometer calibration data */
float tmx, tmy, tmz;
float mag_calib1[12] = {
	/* sphere offset */
	3.2512638, 18.06055698, -4.67724163,
	/* sphere deformation */
	0.99149195, -0.02531768, 0.0042657,
	-0.02531768, 1.00750385, -0.00278795,
	0.0042657, -0.00278795, 1.00173743
};

static void ellipsoid_compensate(float *x, float *y, float *z, float *calib)
{
	tax = *x - calib[0];
	tay = *y - calib[1];
	taz = *z - calib[2];
	*x = tax * calib[3] + tay * calib[4] + taz * calib[5];
	*y = tax * calib[6] + tay * calib[7] + taz * calib[8];
	*z = tax * calib[9] + tay * calib[10] + taz * calib[11];
}


static void askIMU(float *dataspace)
{
	msg_t msg = { 0 };
	memset(&msg, 0, sizeof(msg));
	float *data = (float *)msg.o.raw;

	msg.type = mtRead;

	msg.i.data = NULL;
	msg.i.size = (size_t)0;
	msg.o.data = (void*)NULL;
	msg.o.size = 0;

	msgSend(1, &msg);

	/* accelerometer calibration */
	ellipsoid_compensate(&data[0], &data[1], &data[2], acc_calib1);
	ellipsoid_compensate(&data[0], &data[1], &data[2], acc_finecalib);

	/* magnetometer calibration */
	ellipsoid_compensate(&data[7], &data[8], &data[9], mag_calib1);

	data[0] *= g_scaleerr_common;
	data[1] *= g_scaleerr_common;
	data[2] *= g_scaleerr_common;

	/* gyro niveling */
	data[3] -= gyr_nivel.x;
	data[4] -= gyr_nivel.y;
	data[5] -= gyr_nivel.z;

	memmove(dataspace, data, 10 * sizeof(float));
}

float average_accel(int repeats, unsigned int mswait, const char *axisdir)
{
	float data[10], len = 0;
	int i;
	printf("Position IMU %s and press enter...\n", axisdir);
	getchar();

	for (i = 0; i < repeats; i++) {
		askIMU(data);
		len += sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
		usleep(mswait * 1000);
	}
	return len / repeats;
}

static void accel_fine_tune(void)
{
	unsigned int averaging = 100, mswait = 10;

	vec_t u_plus, u_minus, u_shift;
	u_plus = vec(
		average_accel(averaging, mswait, "X up"),
		average_accel(averaging, mswait, "Y up"),
		average_accel(averaging, mswait, "Z up"));
	u_minus = vec(
		-average_accel(averaging, mswait, "X down"),
		-average_accel(averaging, mswait, "Y down"),
		-average_accel(averaging, mswait, "Z down"));
	u_shift = vec_add(&u_plus, &u_minus);
	u_shift = vec_times(&u_shift, 0.5);

	acc_finecalib[0] = u_shift.x;
	acc_finecalib[1] = u_shift.y;
	acc_finecalib[2] = u_shift.x;

	acc_finecalib[3] = 1.F / (u_plus.x - u_shift.x);
	acc_finecalib[7] = 1.F / (u_plus.y - u_shift.y);
	acc_finecalib[11] = 1.F / (u_plus.z - u_shift.z);

	for (int i = 0; i < 12; i++) {
		printf("%f ", acc_finecalib[i]);
	}
	getchar();
}


void imu_calibrate_acc_gyr_mag(void)
{
	int i, avg = 2000;
	float data[10];
	vec_t a_avg = vec(0, 0, 0), gvec = vec(0, 0, 1), w_avg = vec(0, 0, 0), m_avg = vec(0, 0, 0), x_versor = vec(1, 0, 0), n;
	quat_t iden_q = IDEN_QUAT;

	printf("Calibrating. It wil take 4 seconds...\n");

	for (i = 0; i < avg; i++) {
		askIMU(data);
		a_avg.x += data[0];
		a_avg.y += data[1];
		a_avg.z += data[2];

		w_avg.x += data[3];
		w_avg.y += data[4];
		w_avg.z += data[5];

		m_avg.x += data[7];
		m_avg.y += data[8];
		m_avg.z += data[9];

		usleep(1000 * 5);
	}
	a_avg = vec_times(&a_avg, 0.0005);
	w_avg = vec_times(&w_avg, 0.0005);
	m_avg = vec_times(&m_avg, 0.0005);

	gyr_nivel = w_avg; /* save gyro drift parameters */
	init_m = m_avg;    /* save initial magnetometer reading */

	/* calculate initial rotation */
	n = vec_cross(&a_avg, &init_m);
	init_q = quat_framerot(&a_avg, &n, &gvec, &x_versor, &iden_q);

	/* calculate accelerometer linear deviation from earth g */
	g_scaleerr_common = 1.F / vec_len(&a_avg);

	printf("Calibration done! Begin accel fine tune? (y)\n");
	if (getchar() == 'y') {
		getchar();
		accel_fine_tune();
	}
}

vec_t *imu_measurements(void)
{
	/* get data */
	askIMU(imu_raw);
	meas_data[0] = vec(imu_raw[0], imu_raw[1], imu_raw[2]);
	meas_data[1] = vec(imu_raw[3], imu_raw[4], imu_raw[5]);
	meas_data[2] = vec(imu_raw[7], imu_raw[8], imu_raw[9]);

	return meas_data;
}