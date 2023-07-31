/*
 * Phoenix-Pilot
 *
 * Ekf-specific log module
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#ifndef _EKF_LOG_
#define _EKF_LOG_

#include <stdio.h>
#include <stdint.h>

#include <libsensors.h>

#define MAX_MSG_LEN 60 /* Without terminating NUL character */

#define EKFLOG_SENSC    (1 << 0)
#define EKFLOG_TIME     (1 << 6)

/*
 * Potentially slower implementation, but with no possibility to lose logs.
 *
 * By default log module priorities execution speed over logs consistency.
 * It is possible that not all logs will be stored in result file.
 * It that case appropriate warning is added to file.
 */
#define EKFLOG_STRICT_MODE (1 << 30)


extern int ekflog_timeWrite(time_t timestamp);


extern int ekflog_senscImuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt);


extern int ekflog_senscGpsWrite(const sensor_event_t *gpsEvt);


extern int ekflog_senscBaroWrite(const sensor_event_t *baroEvt);


/* Deinitialize ekflog module */
extern int ekflog_done(void);


/* Initialize log module for `flags` log messages and `path` destination file. Returns 0 on success */
extern int ekflog_init(const char *path, uint32_t flags);


#endif
