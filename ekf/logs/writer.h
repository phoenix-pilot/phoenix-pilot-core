/*
 * Phoenix-Pilot
 *
 * Ekf-specific log writer module
 *
 * Copyright 2022, 2023 Phoenix Systems
 * Authors: Mateusz Niewiadomski, Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#ifndef _EKF_LOG_WRITER_
#define _EKF_LOG_WRITER_

#include <libsensors.h>
#include <matrix.h>
#include <plog.h>


#define EKFLOG_SENSC (1 << 0)
#define EKFLOG_TIME  (1 << 1)
#define EKFLOG_STATE (1 << 2)


/* Logs timestamp */
extern int ekflog_timeWrite(time_t timestamp);


/* Logs data form IMU sensor */
extern int ekflog_imuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt);


/* Logs data from GPS */
extern int ekflog_gpsWrite(const sensor_event_t *gpsEvt);


/* Logs data from barometer */
extern int ekflog_baroWrite(const sensor_event_t *baroEvt);


/* Logs EKF state */
extern int ekflog_stateWrite(const matrix_t *state, time_t timestamp);


/* Initialize log module for `flags` log messages and `path` destination file. Returns 0 on success */
extern int ekflog_writerInit(plog_t *logger, uint32_t flags);


#endif
