/*
 * Phoenix-Pilot
 *
 * Calibration core functions.
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _CALIB_CORE_H_
#define _CALIB_CORE_H_

#include <stdbool.h>
#include <stdio.h>

#include <matrix.h>
#include <vec.h>
#include <libsensors.h>

#include <hmap.h>


#define SENSOR_PATH "/dev/sensors"   /* path to sensor manager device */
#define CALIB_FILE "/etc/calib.conf" /* Path to calibration parameters file */
#define CALIBS_SIZE 16               /* Maximum number of calibrations available. Can be freely increased */

typedef struct {
	char name[16]; /* alias of this calibration */

	union {
		struct {
			int (*run)(void);                /* calibration procedure start */
			int (*done)(void);               /* procedure deinitialization */
			int (*init)(int, const char **); /* procedure initialization */
		} calib;

		struct {
			/* Correction calculation prodecures */
			int (*perform)(sensor_event_t *); /* corrects given measurement event based on own correction type */
			int (*init)(void);                /* initialization of correction algorithm, NULL if unnecessary */
			int (*done)(void);                /* deinitialization of correction algorithm, NULL if unnecessary */
			int (*recalc)(void);              /* correction recalculation procedure */
			const time_t delay;               /* time delay in microseconds between correction recalculation, 0 if correction is time-invariant */
		} corr;
	} proc;

	/* utility related */
	const char *(*help)(void);             /* help message description */
	int (*interpret)(const char *, float); /* calibration file data interpreter */
	int (*write)(FILE *);                  /* calibration file data write */

} calib_t;

/* 
* Reads calibration file, and given a @tag sends preformatted pair param/value 
* to correct calibration procedure interpreter. 
*
* Uses calibration procedures from `calibs` hashmap
*/
int calib_read(const char *path, hmap_t *calibs);

#endif
