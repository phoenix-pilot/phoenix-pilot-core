/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * sensorhub client functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef EKF_SENSORS_CLIENT_H
#define EKF_SENSOR_CLIENT_H

#include <libsensors.h>


/* initializes sensor client that should be accessible under sensorManagerPath (e.g /dev/sensors) */
extern int sensc_init(const char *sensorManagerPath);

/* returns 0 on succesfull acquisition of new imu data from sensorhub, -1 on error */
extern int sensc_imuGet(sensor_event_t *accel_evt, sensor_event_t *gyro_evt, sensor_event_t *mag_evt);

/* returns 0 on succesfull acquisition of new barometer data from sensorhub, -1 on error */
extern int sensc_baroGet(sensor_event_t *baro_evt);

/* returns 0 on succesfull acquisition of new gps data from sensorhub, -1 on error */
extern int sensc_gpsGet(sensor_event_t *gps_evt);

#endif
