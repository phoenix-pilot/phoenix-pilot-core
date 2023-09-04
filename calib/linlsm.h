/*
 * Phoenix-Pilot
 *
 * Linear Least Squares method implementation
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef CALIBTOOL_LINLSM_H_
#define CALIBTOOL_LINLSM_H_

#include <stdint.h>

typedef struct {
	int64_t n;
	double sx;
	double sy;
	double sxx;
	double sxy;
} linlsm_t;

/* produces least square linear fitting to data collected so far. Outputs coefficients and delta */
void linlsm_get(linlsm_t *lsm, float *a, float *b, float *delta);

/* updates linear least square method structure with new data point (x, y) */
void linlsm_update(linlsm_t *lsm, float x, float y);

/* initializes linear least square method structure (no alloc) */
void linlsm_init(linlsm_t *lsm);

#endif
