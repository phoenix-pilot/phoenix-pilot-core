#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <libsensors.h>

#include "sensor_client.h"
#include "tools/rotas_dummy.h"

#define SENSORHUB_PIPES 3 /* number of connections with sensorhub */ 

enum sensor_fd { client_imu, client_baro, client_gps};

struct {
	sensors_data_t *data;
	int fd[SENSORHUB_PIPES]; /* each for one sensor type */
	unsigned char buff[0x400];
} sens_common;


static int sensclient_setupDscr(enum sensor_fd sensor_type, int sensor_type_flag)
{
	sensor_type_t types;
	sensors_ops_t ops = { 0 };

	ioctl(sens_common.fd[sensor_type], SMIOC_SENSORSAVAIL, &types);
	ops.types = types & sensor_type_flag;

	if (ops.types == 0) {
		return -1;
	}

	ioctl(sens_common.fd[sensor_type], SMIOC_SENSORSSET, &ops);

	if ((ops.evtSz * sizeof(sensor_event_t) + sizeof(((sensors_data_t *)0)->size)) > sizeof(sens_common.buff)) {
		fprintf(stderr, "Buff is too small\n");
		return -1;
	}

	return 0;
}


int sensclient_init(const char *sensorManagerPath)
{
	int i = 0;
	unsigned int err = 0;

	/* open file descriptors for all sensor types */
	for (i = 0; i < (sizeof(sens_common.fd) / sizeof(int)); i++) {
		sens_common.fd[i] = open(sensorManagerPath, O_RDWR);
		if (sens_common.fd[i] < 0) {
			err = 1;
			i--;
			break;
		}
	}
	/* if error occured during opening, close all succesfully opened files */
	if (err != 0) {
		while (i >= 0) {
			close(sens_common.fd[i]);
		}
		printf("EKF sensor client: cannot open \"%s\"\n", sensorManagerPath);
		return -1;
	}

	/* ioctl of sensor descriptors */
	err = 0;
	err += sensclient_setupDscr(client_imu, SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG);
	err += sensclient_setupDscr(client_baro, SENSOR_TYPE_BARO);

	if (err != 0) {
		printf("EKF sensor client: cannot setup sensor descriptors\n");
		return -1;
	}

	return 0;
}


int sensclient_sensImu(sensor_event_t *accel_evt, sensor_event_t *gyro_evt, sensor_event_t *mag_evt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sens_common.buff);
	unsigned int flag = SENSOR_TYPE_ACCEL | SENSOR_TYPE_MAG | SENSOR_TYPE_GYRO;
	int j;

	/* read from sensorhub */
	if (read(sens_common.fd[client_imu], sens_common.buff, sizeof(sens_common.buff)) < 0) {
		return -1;
	} 

	/* decompose sensorhub output */
	for (j = 0; (j < data->size) && (flag != 0) ; ++j) {
		switch (data->events[j].type) {
			case SENSOR_TYPE_ACCEL:
				*accel_evt = data->events[j];
				flag &= ~SENSOR_TYPE_ACCEL;
				break;

			case SENSOR_TYPE_MAG:
				*mag_evt = data->events[j];
				flag &= ~SENSOR_TYPE_MAG;
				break;

			case SENSOR_TYPE_GYRO:
				*gyro_evt = data->events[j];
				flag &= ~SENSOR_TYPE_GYRO;
				break;

			default:
				break;
		}
	}

	return (flag == 0) ? 0 : -1;
}


int sensclient_sensBaro(sensor_event_t *baro_evt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sens_common.buff);

	/* read from sensorhub */
	if (read(sens_common.fd[client_baro], sens_common.buff, sizeof(sens_common.buff)) < 0) {
		return -1;
	} 

	if ((data->size > 0) && (data->events[0].type == SENSOR_TYPE_BARO)) {
		*baro_evt = data->events[0];
		return 0;
	}

	return -1;
}


int sensclient_sensGps(sensor_event_t *gps_evt)
{
	sensors_data_t *data;
	data = (sensors_data_t *)(sens_common.buff);

	/* read from sensorhub */
	if (read(sens_common.fd[client_baro], sens_common.buff, sizeof(sens_common.buff)) < 0) {
		return -1;
	} 

	if ((data->size > 0) && (data->events[0].type == SENSOR_TYPE_GPS)) {
		*gps_evt = data->events[0];
		return 0;
	}

	return -1;
}
