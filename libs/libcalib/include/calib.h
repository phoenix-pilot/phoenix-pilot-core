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

#include "hmap.h"


#define SENSOR_PATH "/dev/sensors"   /* path to sensor manager device */
#define CALIB_FILE "/etc/calib.conf" /* Path to calibration parameters file */
#define CALIBS_SIZE 16               /* Maximum number of calibrations available. Can be freely increased */

typedef struct {
	char name[16]; /* alias of this calibration */

	/* process related */
	int (*run)(void);                /* calibration procedure start */
	int (*done)(void);               /* procedure deinitialization */
	int (*init)(int, const char **); /* procedure initialization */

	/* utility related */
	const char *(*help)(void);             /* help message description */
	int (*interpret)(const char *, float); /* calibration file data interpreter */
	int (*write)(FILE *);                  /* calibration file data write */

	/* Correction calculation prodecures */
	int (*corrDo)(sensor_event_t *); /* corrects given measurement event based on own correction type */
	int (*corrInit)(void);           /* initialization of correction algorithm, NULL if unnecessary */
	int (*corrDone)(void);           /* deinitialization of correction algorithm, NULL if unnecessary */
	int (*corrRecalc)(void);         /* correction recalculation procedure */
	const time_t delay;              /* time delay in microseconds between correction recalculation, 0 if correction is time-invariant */
} calib_t;


/* registering new calibration procedure */
extern void calib_register(calib_t *c);

/*
* Reads calibration paramaeters file and writes its data to procedures in `hm`
* Returns 0 on success, -1 if an error occured that prevented read of all  params.
*/
extern int calib_read(const char *path, hmap_t *hm);


extern hmap_t *calib_hashmapGet(void);

#endif
