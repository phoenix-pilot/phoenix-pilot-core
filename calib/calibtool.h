/*
 * Phoenix-Pilot
 *
 * calib.h: drone calibration module
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _CALIBTOOL_CALIB_H_
#define _CALIBTOOL_CALIB_H_

#include <stdbool.h>
#include <stdio.h>

#include <calib.h>


#define SENSOR_PATH "/dev/sensors"

typedef struct {
	char name[16]; /* alias of this calibration */

	/* process related */
	int (*run)(void);                /* calibration procedure start */
	int (*done)(void);               /* procedure deinitialization */
	int (*init)(int, const char **); /* procedure initialization */

	/* utility related */
	const char *(*help)(void);             /* help message description */
	int (*write)(FILE *);                  /* calibration file data write */

	calib_data_t *(*dataGet)(void); /* returns internal structure of libcalib:calib_data_t type */
} calib_ops_t;


/* registering new calibration procedure */
void calib_register(calib_ops_t *c);

#endif
