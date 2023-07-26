/*
 * Phoenix-Pilot
 *
 * sensorhub client functions implementations
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <libsensors.h>

#include "sensc.h"
#include "corr.h"

#define SENSORHUB_PIPES 3       /* number of connections with sensorhub */
#define SECONDS_2_MICRO 1000000 /* number of microseconds in one second */

typedef enum { fd_imuId = 0,
	fd_baroId,
	fd_gpsId } fd_id_t;

struct {
	sensors_data_t *data;
	unsigned char buff[0x400];

	int fdImu;
	int fdBaro;
	int fdGps;

	bool corrEnable;
} sensc_common;


static inline void sensc_closeDescr(int *sensFd) {
	if (*sensFd >= 0) {
		close(*sensFd);
		*sensFd = -1;
	}
}


static int sensc_setupDscr(int sensFd, int typeFlag)
{
	sensor_type_t types;
	sensors_ops_t ops = { 0 };

	ioctl(sensFd, SMIOC_SENSORSAVAIL, &types);
	ops.types = types & typeFlag;

	if (ops.types == 0) {
		return -1;
	}

	ioctl(sensFd, SMIOC_SENSORSSET, &ops);

	if ((ops.evtSz * sizeof(sensor_event_t) + sizeof(((sensors_data_t *)0)->size)) > sizeof(sensc_common.buff)) {
		fprintf(stderr, "Buff is too small\n");
		return -1;
	}

	return 0;
}


int sensc_init(const char *path, bool corrEnable)
{
	bool err;

	sensc_common.corrEnable = corrEnable;
	if (sensc_common.corrEnable == true) {
		if (corr_init() != 0) {
			fprintf(stderr, "Cannot setup correction module\n");
			return -1;
		}
	}

	sensc_common.fdImu = open(path, O_RDWR);
	sensc_common.fdBaro = open(path, O_RDWR);
	sensc_common.fdGps = open(path, O_RDWR);

	err = false;
	err = (sensc_common.fdImu < 0) ? true : err;
	err = (sensc_common.fdBaro < 0) ? true : err;
	err = (sensc_common.fdGps < 0) ? true : err;

	if (err) {
		sensc_closeDescr(&sensc_common.fdImu);
		sensc_closeDescr(&sensc_common.fdBaro);
		sensc_closeDescr(&sensc_common.fdGps);

		fprintf(stderr, "sensc: cannot open sensor descriptors\n");
		return -1;
	}

	/* ioctl of sensor descriptors */
	err = false;
	err = (sensc_setupDscr(fd_imuId, SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG) < 0) ? true : err;
	err = (sensc_setupDscr(fd_baroId, SENSOR_TYPE_BARO) < 0) ? true : err;
	err = (sensc_setupDscr(fd_gpsId, SENSOR_TYPE_GPS) < 0) ? true : err;

	if (err) {
		fprintf(stderr, "sensc: cannot setup sensor descriptors\n");

		if (sensc_common.corrEnable == true) {
			corr_done();
		}

		sensc_closeDescr(&sensc_common.fdImu);
		sensc_closeDescr(&sensc_common.fdBaro);
		sensc_closeDescr(&sensc_common.fdGps);

		return -1;
	}

	return 0;
}


void sensc_deinit(void)
{
	sensc_closeDescr(&sensc_common.fdImu);
	sensc_closeDescr(&sensc_common.fdBaro);
	sensc_closeDescr(&sensc_common.fdGps);

	if (sensc_common.corrEnable == true) {
		corr_done();
	}
}


int sensc_imuGet(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sensc_common.buff);
	unsigned int flag = SENSOR_TYPE_ACCEL | SENSOR_TYPE_MAG | SENSOR_TYPE_GYRO;
	int j;

	/* read from sensorhub */
	if (read(sensc_common.fdImu, sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
		return -1;
	}

	/* decompose sensorhub output */
	for (j = 0; (j < data->size) && (flag != 0); ++j) {
		switch (data->events[j].type) {
			case SENSOR_TYPE_ACCEL:
				*accelEvt = data->events[j];
				flag &= ~SENSOR_TYPE_ACCEL;
				break;

			case SENSOR_TYPE_MAG:
				*magEvt = data->events[j];
				flag &= ~SENSOR_TYPE_MAG;
				break;

			case SENSOR_TYPE_GYRO:
				*gyroEvt = data->events[j];
				flag &= ~SENSOR_TYPE_GYRO;
				break;

			default:
				break;
		}
	}

	if (sensc_common.corrEnable == true) {

		corr_mag(magEvt);
		corr_accorth(accelEvt);
		corr_accrot(accelEvt, gyroEvt, magEvt); /* assumption: IMU returns data in the same frame of reference */
	}

	return (flag == 0) ? 0 : -1;
}


int sensc_baroGet(sensor_event_t *baroEvt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sensc_common.buff);

	/* read from sensorhub */
	if (read(sensc_common.fdBaro, sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
		return -1;
	}

	if ((data->size > 0) && (data->events[0].type == SENSOR_TYPE_BARO)) {
		*baroEvt = data->events[0];
		return 0;
	}

	return -1;
}


int sensc_gpsGet(sensor_event_t *gpsEvt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sensc_common.buff);

	/* read from sensorhub */
	if (read(sensc_common.fdGps, sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
		return -1;
	}

	if ((data->size > 0) && (data->events[0].type == SENSOR_TYPE_GPS)) {
		*gpsEvt = data->events[0];
		return 0;
	}

	return -1;
}


int sensc_timeGet(time_t *time)
{
	struct timeval tv;

	if (gettimeofday(&tv, NULL) != 0) {
		return -1;
	}

	*time = SECONDS_2_MICRO * tv.tv_sec + tv.tv_usec;

	return 0;
}
