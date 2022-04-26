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

float raw[10];
vec_t meas_data[2];

static void askIMU(float *dataspace)
{
	msg_t msg = { 0 };
	memset(&msg, 0, sizeof(msg));
	volatile float *data = (float *)msg.o.raw;
	float tx, ty, tz;

	msg.type = mtRead;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.size = NULL;
	msg.o.size = 0;

	msgSend(1, &msg);

	/* accelerometer calibration */
	tx = data[0] + 0.20429579;
	ty = data[1] + 0.00556169;
	tz = data[2] + 0.00156721;
	data[0] = 1.00248224e+00 * tx + 1.30518466e-04 * ty - 1.12134335e-04 * tz;
	data[1] = 1.30518466e-04 * tx + 9.98607692e-01 * ty + 3.40661521e-03 * tz;
	data[2] = -1.12134335e-04 * tx + 3.40661521e-03 * ty + 9.98926355e-01 * tz;

	data[3] -= gyr_nivel.x;
	data[4] -= gyr_nivel.y;
	data[5] -= gyr_nivel.z;

	memmove(dataspace, data, 10 * sizeof(float));
}

void imu_calibrate_acc_gyr(void)
{
	int i, avg = 1000;
	float data[10];
	vec_t a_avg = vec(0, 0, 0), gvec = vec(0, 0, 1), w_avg = vec(0, 0, 0);

	for (i = 0; i < avg; i++) {
		askIMU(data);
		a_avg.x += data[0];
		a_avg.y += data[1];
		a_avg.z += data[2];
		w_avg.x += data[3];
		w_avg.y += data[4];
		w_avg.z += data[5];
		usleep(1000 * 5);
	}
	a_avg = vec_scl(&a_avg, 0.001);
	w_avg = vec_scl(&w_avg, 0.001);

	init_q = quat_vec2vec(&a_avg, &gvec);
	gyr_nivel = w_avg;

	quat_print(&init_q);
	getchar();
}

vec_t *imu_measurements(void)
{
	/* get data */
	askIMU(raw);
	meas_data[0] = vec(raw[0], raw[1], raw[2]);
	meas_data[1] = vec(raw[3], raw[4], raw[5]);


	return meas_data;
}