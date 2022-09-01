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


#define SENSOR_PATH "/dev/sensors"

typedef struct _calib_t {
	char name[16]; /* alias of this calibration */

	/* process related */
	int (*run)(void);                /* calibration procedure start */
	int (*done)(void);               /* procedure deinitialization */
	int (*init)(int, const char **); /* procedure initialization */

	/* utility related */
	const char *(*help)(void);                        /* help message description */
	int (*interpret)(const char *valName, float val); /* calibration file data interpreter */
	int (*write)(FILE *file);                         /* calibration file data write */

	struct _calib_t *next; /* linked list pointer */
} calib_t;


/* registering new calibration procedure */
void calib_register(calib_t *c);

#endif
