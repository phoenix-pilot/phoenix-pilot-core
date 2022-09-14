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

#define NUM_OF_MOTORS 4


typedef enum { typeMagmot = 0, typeMagiron } calibType_t;


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
	} params;
} calib_t;


/* read calibration file pointed by 'path' searching for calibration named `tag` and saving its content to 'cal' */
int calib_readFile(const char *path, calibType_t type, calib_t *cal);


#endif
