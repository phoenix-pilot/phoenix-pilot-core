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

#ifndef EKF_SENSOR_CLIENT_H
#define EKF_SENSOR_CLIENT_H

#include <libsensors.h>

#include "sensor_client.h"


void sensclient_sensImu(sensor_event_t *accel_evt, sensor_event_t *gyro_evt, sensor_event_t *mag_evt);

void sensclient_sensBaro(sensor_event_t *baro_evt);

void sensclient_sensGps(sensor_event_t *gps_evt);

#endif
