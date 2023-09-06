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
#include <matrix.h>


/*
 * Reads next timestamp log. In case of a success returns 0.
 * If end-of-file is encountered returns EOF.
 * In case of an error returns EOF and sets appropriate errno value.
 */
extern int ekflog_timeRead(time_t *timestamp);


/*
 * Reads next IMU log. In case of a success returns 0.
 * If end-of-file is encountered returns EOF.
 * In case of an error returns EOF and sets appropriate errno value.
 */
extern int ekflog_imuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt);


/*
 * Reads next GPS log. In case of a success returns 0.
 * If end-of-file is encountered returns EOF.
 * In case of an error returns EOF and sets appropriate errno value.
 */
extern int ekflog_gpsRead(sensor_event_t *gpsEvt);


/*
 * Reads next barometer log. In case of a success returns 0.
 * If end-of-file is encountered returns EOF.
 * In case of an error returns EOF and sets appropriate errno value.
 */
extern int ekflog_baroRead(sensor_event_t *baroEvt);


/* Reads EKF state, State matrix have to be initiated with correct size. */
extern int ekflog_stateRead(matrix_t *state, time_t *timestamp);


/* Initiates module, `path` must leads to binary ekf logs file. On success returns 0. */
extern int ekflog_readerInit(const char *path);


/* Deinitialize module. On success returns 0. */
extern int ekflog_readerDone(void);


#endif
