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

#define EKFLOG_SENSC    (1 << 0)
#define EKFLOG_MEAS     (1 << 1)
#define EKFLOG_EKF_IMU  (1 << 2)
#define EKFLOG_EKF_POS  (1 << 3)
#define EKFLOG_GPS_POS  (1 << 4)
#define EKFLOG_GPS_MEAS (1 << 5)


/* Prints `flags` type log message passed as `format` */
extern int ekflog_write(uint32_t flags, const char *format, ...);


/* Deinitialize ekflog module */
extern int ekflog_done(void);


/* Initialize log module for `flags` log messages and `path` destination file. Returns 0 on success */
extern int ekflog_init(const char *path, uint32_t flags);


#endif
