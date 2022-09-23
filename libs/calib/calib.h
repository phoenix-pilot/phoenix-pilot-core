/*
 * Phoenix-Pilot
 *
 * Calibration library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _LIBCALIB_CALIB_H_
#define _LIBCALIB_CALIB_H_

#include <matrix.h>

#define CALIB_PATH    "/etc/calib.conf"
#define NUM_OF_MOTORS 4

#define MAGMOT_TAG             "magmot"
#define MAGMOT_PARAMS          36
#define MAGMOT_CUTOFF_THROTTLE 0.3 /* minimal throttle value calibration is made with */

#define MAGIRON_TAG     "magiron"
#define MAGIRON_PARAMS  12
#define CHAR_SOFTIRON   's'
#define CHAR_HARDIRON   'h'
#define SOFTCAL_ROWSPAN 3
#define SOFTCAL_COLSPAN 3
#define HARDCAL_ROWSPAN 3
#define HARDCAL_COLSPAN 1

#define MOTLIN_TAG    "motlin"
#define MOTLIN_PARAMS 8


typedef enum { typeMagmot = 0, typeMagiron, typeMotlin } calibType_t;


typedef struct {
	calibType_t type;
	union {
		struct {
			matrix_t softCal; /* 3x3 matrix for soft iron calib. parameters. */
			matrix_t hardCal; /* 3x1 matrix for hard iron calibration */
		} magiron;

		struct {
			float motorEq[NUM_OF_MOTORS][3][3]; /* motorEq[motorId 0/1/2...NUM_OF_MOTORS][axisId x/y/z][equation_param a/b/c] */
		} magmot;

		struct {
			float motorEq[NUM_OF_MOTORS][2]; /* motorEq[motorId 0/1/2...NUM_OF_MOTORS][equation_parameter a/b] */
		} motlin;
	} params;
} calib_data_t;


/*
* Read calibration file pointed by 'path' searching for calibration named `tag` and saving its content to 'cal'.
* If 'path' does not point to a file default values are written and 0 is returned (success).
*/
int calib_readFile(const char *path, calibType_t type, calib_data_t *cal);


/* Deallocates all memory used by 'cal' */
void calib_free(calib_data_t *cal);


#endif
