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

	int corrInitFlags;
} sensc_common;


static int sensc_setupDescr(int sensFd, int typeFlag)
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


static inline void sensc_closeDescr(int *sensFd)
{
	if (*sensFd >= 0) {
		close(*sensFd);
		*sensFd = -1;
	}
}


static int sensc_openDescr(const char *path, int typeFlag, int initFlags, int *sensFd)
{
	int sensorType;

	if ((initFlags & typeFlag) == 0) {
		*sensFd = -1;
		return 0;
	}

	switch (typeFlag) {
		case SENSC_INIT_IMU:
			sensorType = SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG;
			break;

		case SENSC_INIT_BARO:
			sensorType = SENSOR_TYPE_BARO;
			break;

		case SENSC_INIT_GPS:
			sensorType = SENSOR_TYPE_GPS;
			break;

		default:
			fprintf(stderr, "sensc: unknown type\n");
			*sensFd = -1;
			return -1;
	}

	*sensFd = open(path, O_RDWR);
	if (*sensFd < 0) {
		fprintf(stderr, "sensc: open failed\n");
		*sensFd = -1;
		return -1;
	}

	if (sensc_setupDescr(*sensFd, sensorType) < 0) {
		fprintf(stderr, "sensc: ioctl failed\n");
		close(*sensFd);
		*sensFd = -1;
		return -1;
	}

	return 0;
}


int sensc_init(const char *path, int corrInitFlags, int sensInitFlags)
{
	int err;

	sensc_common.corrInitFlags = corrInitFlags;
	if (corr_init(corrInitFlags) != 0) {
		fprintf(stderr, "Cannot setup correction module\n");
		return -1;
	}

	err = 0;
	err |= sensc_openDescr(path, SENSC_INIT_IMU, sensInitFlags, &sensc_common.fdImu);
	err |= sensc_openDescr(path, SENSC_INIT_BARO, sensInitFlags, &sensc_common.fdBaro);
	err |= sensc_openDescr(path, SENSC_INIT_GPS, sensInitFlags, &sensc_common.fdGps);

	if (err != 0) {
		fprintf(stderr, "sensc: init failed\n");
		sensc_closeDescr(&sensc_common.fdImu);
		sensc_closeDescr(&sensc_common.fdBaro);
		sensc_closeDescr(&sensc_common.fdGps);

		corr_done();
		return -1;
	}

	return 0;
}


void sensc_deinit(void)
{
	sensc_closeDescr(&sensc_common.fdImu);
	sensc_closeDescr(&sensc_common.fdBaro);
	sensc_closeDescr(&sensc_common.fdGps);

	corr_done();
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

	/* Corrections */
	corr_imu(accelEvt, gyroEvt, magEvt);

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
