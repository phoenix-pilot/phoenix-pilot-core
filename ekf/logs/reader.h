/*
 * Phoenix-Pilot
 *
 * Ekf-specific log reader module
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __EKF_LOG_READER_H__
#define __EKF_LOG_READER_H__


#include <libsensors.h>


/* Reads next timestamp log */
extern int ekflog_timeRead(time_t *timestamp);


/* Reads next IMU log */
extern int ekflog_imuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt);


/* Reads next GPS log */
extern int ekflog_gpsRead(sensor_event_t *gpsEvt);


/* Reads next barometer log */
extern int ekflog_baroRead(sensor_event_t *baroEvt);


/* Initiates module, `path` must leads to binary ekf logs file. On success returns 0. */
extern int ekflog_readerInit(const char *path);


/* Deinitialize module. On success returns 0. */
extern int ekflog_readerDone(void);


#endif
