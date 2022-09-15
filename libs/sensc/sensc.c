/*
 * Phoenix-Pilot
 *
 * extended kalman filter
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
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <libsensors.h>

#include "sensc.h"
#include "corr.h"

#define SENSORHUB_PIPES 3 /* number of connections with sensorhub */

typedef enum { fd_imuId = 0, fd_baroId, fd_gpsId } fd_id_t;

struct {
	sensors_data_t *data;
	int fd[SENSORHUB_PIPES]; /* each for one sensor type */
	unsigned char buff[0x400];
} sensc_common;


static int sensc_setupDscr(fd_id_t type, int typeFlag)
{
	sensor_type_t types;
	sensors_ops_t ops = { 0 };

	ioctl(sensc_common.fd[type], SMIOC_SENSORSAVAIL, &types);
	ops.types = types & typeFlag;

	if (ops.types == 0) {
		return -1;
	}

	ioctl(sensc_common.fd[type], SMIOC_SENSORSSET, &ops);

	if ((ops.evtSz * sizeof(sensor_event_t) + sizeof(((sensors_data_t *)0)->size)) > sizeof(sensc_common.buff)) {
		fprintf(stderr, "Buff is too small\n");
		return -1;
	}

	return 0;
}


int sensc_init(const char *path)
{
	int i = 0;
	unsigned int err = 0;

	if (corr_init() != 0) {
		fprintf(stderr, "Cannot setup correction module\n");
		return -1;
	}

	/* open file descriptors for all sensor types */
	for (i = 0; i < (sizeof(sensc_common.fd) / sizeof(int)); i++) {
		sensc_common.fd[i] = open(path, O_RDWR);
		if (sensc_common.fd[i] < 0) {
			err = 1;
			i--;
			break;
		}
	}
	/* if error occured during opening, close all succesfully opened files */
	if (err != 0) {
		fprintf(stderr, "sensc: cannot open \"%s\"\n", path);

		corr_done();
		while (i >= 0) {
			close(sensc_common.fd[i]);
			i--;
		}

		return -1;
	}

	/* ioctl of sensor descriptors */
	err = 0;
	err += sensc_setupDscr(fd_imuId, SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG);
	err += sensc_setupDscr(fd_baroId, SENSOR_TYPE_BARO);
	err += sensc_setupDscr(fd_gpsId, SENSOR_TYPE_GPS);

	if (err != 0) {
		fprintf(stderr, "sensc: cannot setup sensor descriptors\n");

		corr_done();
		for (i = 0; i < (sizeof(sensc_common.fd) / sizeof(int)); i++) {
			close(sensc_common.fd[i]);
		}

		return -1;
	}

	return 0;
}


void sensc_deinit(void)
{
	int i;

	for (i = 0; i < (sizeof(sensc_common.fd) / sizeof(int)); i++) {
		close(sensc_common.fd[i]);
	}

	corr_done();
}


int sensc_imuGet(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sensc_common.buff);
	unsigned int flag = SENSOR_TYPE_ACCEL | SENSOR_TYPE_MAG | SENSOR_TYPE_GYRO;
	int j;

	/* read from sensorhub */
	if (read(sensc_common.fd[fd_imuId], sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
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

	return (flag == 0) ? 0 : -1;
}


int sensc_baroGet(sensor_event_t *baroEvt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sensc_common.buff);

	/* read from sensorhub */
	if (read(sensc_common.fd[fd_baroId], sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
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
	if (read(sensc_common.fd[fd_baroId], sensc_common.buff, sizeof(sensc_common.buff)) < 0) {
		return -1;
	}

	if ((data->size > 0) && (data->events[0].type == SENSOR_TYPE_GPS)) {
		*gpsEvt = data->events[0];
		return 0;
	}

	return -1;
}
