/*
 * Phoenix-Pilot
 *
 * Iterative statistics functions for series:
 *  - maxima/minima/sum
 *  - variance (using Welford method)
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <statistics.h>


double stats_variance(stats_t *stats)
{
	return (stats->n <= 1) ? 0 : stats->priv[1] / (stats->n - 1);
}


double stats_mean(stats_t *stats)
{
	return (stats->n == 0) ? 0 : stats->sum / stats->n;
}


void stats_update(stats_t *stats, double sample)
{
	double old;

	/* No float.h with DBL_MAX/MIN - using counter as first assignment indicator */
	if (stats->n == 0) {
		stats->max = sample;
		stats->min = sample;
	}
	else {
		if (sample > stats->max) {
			stats->max = sample;
		}
		else if (sample < stats->min) {
			stats->min = sample;
		}
	}

	stats->sum += sample;
	stats->n++;

	/* Variance calculations using Welford method */
	old = stats->priv[0];
	stats->priv[0] = stats->priv[0] + (sample - stats->priv[0]) / stats->n;
	stats->priv[1] = stats->priv[1] + (sample - stats->priv[0]) * (sample - old);
}


void stats_reset(stats_t *stats)
{
	/* No float.h with DBL_MAX/MIN */
	stats->max = 0;
	stats->min = 0;
	stats->sum = 0;
	stats->n = 0;

	stats->priv[0] = 0;
	stats->priv[1] = 0;
}
