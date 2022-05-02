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

float imu_raw[10];
vec_t meas_data[3];

static void askIMU(float *dataspace)
{
	msg_t msg = { 0 };
	memset(&msg, 0, sizeof(msg));
	volatile float *data = (float *)msg.o.raw;
	float tx, ty, tz, tmx, tmy, tmz;

	msg.type = mtRead;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.size = NULL;
	msg.o.size = 0;

	msgSend(1, &msg);

	/* accelerometer calibration */
	tx = data[0] + 0.01878438;
	ty = data[1] + 0.01529312;
	tz = data[2] - 0.00847015;
	data[0] = 1.00314301e+00 * tx + 3.43707788e-03 * ty -3.07747253e-03 * tz;
	data[1] = 3.43707788e-03 * tx + 9.99766169e-01 * ty + 5.66230781e-04 * tz;
	data[2] = -3.07747253e-03 * tx + 5.66230781e-04 * ty + 9.97121509e-01 * tz;

	/* gyro niveling */
	data[3] -= gyr_nivel.x;
	data[4] -= gyr_nivel.y;
	data[5] -= gyr_nivel.z;

	/* magnetometer calibration */
	tmx = data[7] - (3.2512638);
	tmy = data[8] - (18.06055698);
	tmz = data[9] - (-4.67724163);

	data[7] =  0.99149195 * tmx -0.02531768 * tmy + 0.0042657 * tmz;
	data[8] = -0.02531768 * tmx + 1.00750385 * tmy -0.00278795 * tmz;
	data[9] =  0.0042657 * tmx -0.00278795 * tmy + 1.00173743 * tmz;

	memmove(dataspace, data, 10 * sizeof(float));
}

void imu_calibrate_acc_gyr_mag(void)
{
	int i, avg = 2000;
	float data[10];
	vec_t a_avg = vec(0, 0, 0), gvec = vec(0, 0, 1), w_avg = vec(0, 0, 0), m_avg = vec(0, 0, 0), x_versor = vec(1, 0, 0), n;
	quat_t iden_q = IDEN_QUAT;

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
	init_m = m_avg; /* save initial magnetometer reading */

	/* calculate initial rotation */
	n = vec_cross(&a_avg, &init_m);
	init_q = quat_framerot(&a_avg, &n, &gvec, &x_versor, &iden_q);

	quat_print(&init_q);
	quat_print(&gyr_nivel);
	getchar();
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