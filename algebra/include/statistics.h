/*
 * Phoenix-Pilot
 *
 * statistics algebra library
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PHOENIX_STATISTICS_H__
#define __PHOENIX_STATISTICS_H__


typedef struct {
	unsigned long long n; /* length of current series */
	double max;           /* maximum value recorded in current series */
	double min;           /* minimum value recorded in current series */
	double sum;           /* sum of current series */

	double priv[2]; /* variance calculations private variables */
} stats_t;


/* Returns variance of current series. Returns 0 if stdev cannot be calculated */
extern double stats_variance(stats_t *stats);


/* Returns average of current series */
extern double stats_mean(stats_t *stats);


/* iteratively updates `stats` limits structure with new `sample` */
extern void stats_update(stats_t *stats, double sample);


/* Resets/initializes `stats` limits structure */
extern void stats_reset(stats_t *stats);


#endif
