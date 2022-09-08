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

#include <matrix.h>
#include <vec.h>
#include <libsensors.h>


#define SENSOR_PATH "/dev/sensors"

/* Maximum number of calibrations available. Can be freely increased */
#define CALIBS_SIZE 16


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

	/* Correction calculation prodecures */
	int (*cDo)(sensor_event_t *evt); /* corrects given measurement event based on own correction type */
	int (*cInit)(void);              /* initialization of correction algorithm, NULL if unnecessary */
	int (*cDone)(void);              /* deinitialization of correction algorithm, NULL if unnecessary */
	int (*cRecalc)(void);            /* correction recalculation procedure */
	const time_t delay;                 /* time delay in microseconds between correction recalculation, 0 if correction is time-invariant */
} calib_t;


/* registering new calibration procedure */
extern void calib_register(calib_t *c);

#endif
