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


static void askIMU(float * dataspace)
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
	for (int i = 0; i < 100; i++)
	{
		askIMU(data);
		acc.x += data[0];
		acc.y += data[1];
		acc.z += data[2];
		gyr.x += data[3];
		gyr.y += data[4];
		gyr.z += data[5];
		usleep(1000 * 50);
	}
	/* average collected data */
	acc = vec_scl(&acc, 0.01);
	gyr = vec_scl(&gyr, 0.01);

	/* calculate acceleration rotation quaternion */
	vec_t g_versor = vec(0, 0, 1);
	vec_t acc_unit = vec(acc.x, acc.y, acc.z);
	vec_normalize(&acc_unit);
	quat_t qi = quat_vec2vec(&acc_unit, &g_versor);
	float scale = 1.F / vec_len(&acc);
	printf("scale: %f\n", scale);

	/* calculate gyroscope offset vector */
	vec_t gyr_offset = vec_scl(&gyr, -1);

	for (int i=0 ; i<10; i++) {
		usleep(1000*500);
		askIMU(data);
		acc.x = data[0];
		acc.y = data[1];
		acc.z = data[2];
		gyr.x = data[3];
		gyr.y = data[4];
		gyr.z = data[5];

		acc = vec_scl(&acc, scale);
		quat_vecrot(&acc, &qi);

		gyr = vec_add(&gyr, &gyr_offset);

		quat_print(&acc);
		quat_print(&gyr);
		printf("\n");
	}
}