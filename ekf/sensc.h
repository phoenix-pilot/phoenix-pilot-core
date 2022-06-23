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


/* initializes sensor client that should be accessible under path (e.g /dev/sensors) */
extern int sensc_init(const char *path);

/* returns 0 on succesfull acquisition of new imu data from sensorhub, -1 on error */
extern int sensc_imuGet(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt);

/* returns 0 on succesfull acquisition of new barometer data from sensorhub, -1 on error */
extern int sensc_baroGet(sensor_event_t *baroEvt);

/* returns 0 on succesfull acquisition of new gps data from sensorhub, -1 on error */
extern int sensc_gpsGet(sensor_event_t *gpsEvt);

#endif
