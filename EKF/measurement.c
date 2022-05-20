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

#include "kalman_implem.h"

#include <imx6ull-sensi2c.h>
#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

float g_scaleerr_common = 1;

struct sens_imu_t imuSensor;
struct sens_baro_t baroSensor;
struct sens_mag_t magSensor;

/* accelerometer calibration data */
float tax, tay, taz;
float acc_calib1[12] = {
	/* sphere offset */
	0.01737944, -0.01926739, 0.00982283,
	/* sphere deformation */
	1.00138962, 0.00186323, -0.00175369,
	0.00186323, 0.99879068, 0.00357516,
	-0.00175369, 0.00357516, 0.99984077
};
float acc_calib2[12] = {
	/* sphere offset */
	0.00116779, -0.00139108, -0.00371815,
	/* sphere deformation */
	1.00284, -0.00287202, -0.00229445,
	-0.00287202, 0.99934558, 0.00263417,
	-0.00229445, 0.00263417, 0.9978414
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


void imu_calibrate_acc_gyr_mag(void)
{
	int i, avg = 2000, press_avg = 0;
	float press_calib = 0, temp_calib = 0;
	vec_t a_avg = vec(0, 0, 0), gvec = vec(0, 0, 1), w_avg = vec(0, 0, 0), m_avg = vec(0, 0, 0), x_versor = vec(1, 0, 0), n;
	quat_t iden_q = IDEN_QUAT;

	printf("Calibrating. It wil take 4 seconds...\n");

	for (i = 0; i < avg; i++) {
		sensImu(&imuSensor);
		sensMag(&magSensor);
		a_avg.x += imuSensor.accel_x;
		a_avg.y += imuSensor.accel_y;
		a_avg.z += imuSensor.accel_z;

		w_avg.x += imuSensor.gyr_x;
		w_avg.y += imuSensor.gyr_y;
		w_avg.z += imuSensor.gyr_z;

		m_avg.x += magSensor.mag_x;
		m_avg.y += magSensor.mag_y;
		m_avg.z += magSensor.mag_z;

		if (sensBaro(&baroSensor) > 0) {
			press_calib += baroSensor.press;
			temp_calib += baroSensor.baro_temp;
			press_avg++;
		}

		usleep(1000 * 5);
	}
	a_avg = vec_times(&a_avg, 0.0005);
	w_avg = vec_times(&w_avg, 0.0005);
	m_avg = vec_times(&m_avg, 0.0005);
	press_calib /= press_avg;
	temp_calib /= press_avg;

	gyr_nivel = w_avg;           /* save gyro drift parameters */
	init_m = m_avg;              /* save initial magnetometer reading */
	base_pressure = press_calib; /* save initial pressure */
	base_temp = temp_calib;      /* save initial temperature */

	/* calculate initial rotation */
	n = vec_cross(&a_avg, &init_m);
	init_q = quat_framerot(&a_avg, &n, &gvec, &x_versor, &iden_q);

	/* calculate accelerometer linear deviation from earth g */
	g_scaleerr_common = 1.F / vec_len(&a_avg);
}

void acquireImuMeasurements(vec_t *accels, vec_t *gyros, vec_t *mags)
{
	sensImu(&imuSensor);
	sensMag(&magSensor);

	/* accelerometer calibration */
	ellipsoid_compensate(&imuSensor.accel_x, &imuSensor.accel_y, &imuSensor.accel_z, acc_calib1);
	ellipsoid_compensate(&imuSensor.accel_x, &imuSensor.accel_y, &imuSensor.accel_z, acc_calib2);

	/* magnetometer calibration */
	ellipsoid_compensate(&magSensor.mag_x, &magSensor.mag_y, &magSensor.mag_z, mag_calib1);

	/* gyro niveling */
	imuSensor.gyr_x -= gyr_nivel.x;
	imuSensor.gyr_y -= gyr_nivel.y;
	imuSensor.gyr_z -= gyr_nivel.z;

	if (accels != NULL && gyros != NULL && mags != NULL) {
		/* write data */
		*accels = vec(imuSensor.accel_x, imuSensor.accel_y, imuSensor.accel_z);
		*gyros = vec(imuSensor.gyr_x, imuSensor.gyr_y, imuSensor.gyr_z);
		*mags = vec(magSensor.mag_x, magSensor.mag_y, magSensor.mag_z);
	}
}


int acquireBaroMeasurements(float *pressure, float *temperature, float *dtBaroUs)
{
	struct timeval lastT = baroSensor.timestamp;

	if (sensBaro(&baroSensor) > 0) {
		*pressure = baroSensor.press;
		*temperature = baroSensor.baro_temp;
		*dtBaroUs = (baroSensor.timestamp.tv_sec - lastT.tv_sec) * 1000000 + baroSensor.timestamp.tv_usec - lastT.tv_usec;
		return 0;
	}
	return -1;
}
