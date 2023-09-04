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

#include <stdint.h>
#include <stdlib.h>

#include "linlsm.h"

/* produces least square linear fitting to data collected so far. Outputs coefficients and delta */
void linlsm_get(linlsm_t *lsm, float *a, float *b, float *delta)
{
	float deltaLocal, aLocal, bLocal;

	deltaLocal = lsm->n * lsm->sxx - (lsm->sx * lsm->sx);
	aLocal = (float)(lsm->n * lsm->sxy - lsm->sx * lsm->sy) / deltaLocal;
	bLocal = (float)(lsm->sxx * lsm->sy - lsm->sx * lsm->sxy) / deltaLocal;

	if (a != NULL) {
		*a = aLocal;
	}

	if (b != NULL) {
		*b = bLocal;
	}

	if (delta != NULL) {
		*delta = deltaLocal;
	}
}


/* updates linear least square method structure with new data point (x, y) */
void linlsm_update(linlsm_t *lsm, float x, float y)
{
	lsm->n++;
	lsm->sx += x;
	lsm->sy += y;
	lsm->sxx += x * x;
	lsm->sxy += x * y;
}


/* initializes linear least square method structure (no alloc) */
void linlsm_init(linlsm_t *lsm)
{
	lsm->n = 0;
	lsm->sx = 0;
	lsm->sy = 0;
	lsm->sxx = 0;
	lsm->sxy = 0;
}
