/*
 * Phoenix-Pilot
 *
 * Ekf sensor log reader
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __LOG_READER_H__
#define __LOG_READER_H__


#include <sensors.h>


/* clang-format off */
typedef enum { timestampLog = 0, senscLog } logType_t;
/* clang-format on */

typedef struct
{
	logType_t type;

	union {
		time_t timestamp;
		sensor_event_t sensc;
	} data;
} logReader_data_t;


/* Initiates module, `path` must leads to binary ekf logs file. On success returns 0. */
extern int logReader_init(const char *path);


/* Deinitialize module. On success returns 0. */
extern int logReader_done();


/* On success returns 0 and `result` is equal to next entry from log file. */
extern int logReader_nextEntry(logReader_data_t *result);


#endif
