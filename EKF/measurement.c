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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/msg.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

quat_t nivel_accel;
float scale_acc, raw[10];
vec_t meas_data[2], nivel_gyro;

static void askIMU(float *dataspace)
{
	msg_t msg = { 0 };
	memset(&msg, 0, sizeof(msg));
	volatile float *data = (float *)msg.o.raw;

	msg.type = mtRead;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.size = NULL;
	msg.o.size = 0;

	msgSend(1, &msg);

	memmove(dataspace, data, 10 * sizeof(float));
}


void imu_calibrate_acc_gyr(void)
{
	float data[10];
	vec_t acc = vec(0, 0, 0);
	vec_t gyr = vec(0, 0, 0);

	printf("calibrating accel/gyr...\n");
	/* collect sample data of STATIONARY imu */
	for (int i = 0; i < 1000; i++) {
		askIMU(data);
		acc.x += data[0];
		acc.y += data[1];
		acc.z += data[2];
		gyr.x += data[3];
		gyr.y += data[4];
		gyr.z += data[5];
		usleep(1000 * 5);
	}
	/* average collected data */
	acc = vec_scl(&acc, 0.001);
	gyr = vec_scl(&gyr, 0.001);

	/* calculate acceleration rotation quaternion */
	vec_t g_versor = vec(0, 0, 1);
	vec_t acc_unit = vec(acc.x, acc.y, acc.z);
	vec_normalize(&acc_unit);
	nivel_accel = quat_vec2vec(&acc_unit, &g_versor);
	init_q = quat(nivel_accel.a, nivel_accel.i, nivel_accel.j, nivel_accel.k);
	scale_acc = 1.F / vec_len(&acc);

	/* calculate gyroscope offset vector */
	vec_t nivel_gyro = vec_scl(&gyr, -1);

	// for (int i=0 ; i<100; i++) {
	// 	usleep(1000*50);
	// 	askIMU(data);
	// 	acc.x = data[0];
	// 	acc.y = data[1];
	// 	acc.z = data[2];
	// 	gyr.x = data[3];
	// 	gyr.y = data[4];
	// 	gyr.z = data[5];

	// 	acc = vec_scl(&acc, scale_acc);
	// 	quat_vecrot(&acc, &init_q);

	// 	gyr = vec_add(&gyr, &nivel_gyro);

	// 	printf("%f %f %f %f %f %f\n", acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
	// }
}

vec_t *imu_measurements(void)
{
	/* get data */
	askIMU(raw);
	meas_data[0] = vec(raw[0], raw[1], raw[2]);
	meas_data[1] = vec(raw[3], raw[4], raw[5]);

	meas_data[0] = vec_scl(&(meas_data[0]), scale_acc);
	meas_data[1] = vec_add(&(meas_data[1]), &nivel_gyro);

	return meas_data;
}