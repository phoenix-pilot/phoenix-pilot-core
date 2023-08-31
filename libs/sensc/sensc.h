/*
 * Phoenix-Pilot
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

#ifndef __SENSORS_CLIENT_H__
#define __SENSORS_CLIENT_H__

#include <stdbool.h>

#include <libsensors.h>

#define SENSC_INIT_IMU  (1 << 0)
#define SENSC_INIT_BARO (1 << 1)
#define SENSC_INIT_GPS  (1 << 2)


#define CORR_ENBL_MAGIRON (1 << 0)
#define CORR_ENBL_MAGMOT  (1 << 1)
#define CORR_ENBL_ACCORTH (1 << 2)
#define CORR_ENBL_ACCROT  (1 << 3)
#define CORR_ENBL_TEMPIMU (1 << 4)

#define CORR_ENBL_ALL  (~(unsigned int)0)
#define CORR_ENBL_NONE (0)


/* initializes sensor client that should be accessible under path (e.g /dev/sensors) with option to turn on/off corrections module */
extern int sensc_init(const char *path, bool corrEnable, int initFlags);

/* deinitializes all initialized parts of sensor client */
extern void sensc_deinit(void);

/* returns 0 on successful acquisition of new imu data from sensorhub, -1 on error */
extern int sensc_imuGet(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt);

/* returns 0 on successful acquisition of new barometer data from sensorhub, -1 on error */
extern int sensc_baroGet(sensor_event_t *baroEvt);

/* returns 0 on successful acquisition of new gps data from sensorhub, -1 on error */
extern int sensc_gpsGet(sensor_event_t *gpsEvt);

/* returns 0 on successful acquisition of time in microseconds, -1 on error */
extern int sensc_timeGet(time_t *time);

#endif
