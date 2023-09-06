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


#define EKFLOG_SENSC (1 << 0)
#define EKFLOG_TIME  (1 << 1)
#define EKFLOG_STATE (1 << 2)

/*
 * Potentially slower implementation, but with no possibility to lose logs.
 *
 * By default log module priorities execution speed over logs consistency.
 * It is possible that not all logs will be stored in result file.
 * It that case appropriate warning is added to file.
 */
#define EKFLOG_STRICT_MODE (1 << 30)


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


/* Deinitialize ekflog writer module */
extern int ekflog_writerDone(void);


/* Initialize log module for `flags` log messages and `path` destination file. Returns 0 on success */
extern int ekflog_writerInit(const char *path, uint32_t flags);


#endif
