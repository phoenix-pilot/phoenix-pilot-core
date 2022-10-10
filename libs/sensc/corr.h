/*
 * Phoenix-Pilot
 *
 * sensorhub client correction header
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _SENSC_CORR_H_
#define _SENSC_CORR_H_

#include <calib.h>
#include <libsensors.h>


/* Performs calibration on magnetometer data */
void corr_mag(sensor_event_t *magEvt);


/* performs initial data rotation based on accelerometer initial attitude quaternion */
void corr_accrot(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt);


/* deinitializes correction procedures */
void corr_done(void);


/* initializes all correction procedures. Returns 0 on success */
int corr_init(void);

#endif
