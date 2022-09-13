/*
 * Phoenix-Pilot
 *
 * main.c: Drone corrections library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include <calibcore.h>
#include <hmap.h>

#include "corr.h"


typedef struct {
	calib_t *cal;
	time_t deadline;
} corr_sched_t;


struct {
	hmap_t *corrs;

	corr_sched_t *sched;
	unsigned int schedSz;

	char stack[4096];
	volatile bool run;
} corr_common;


/* Register new calib_t procedure. */
void corr_register(calib_t *c)
{
	if (hmap_insert(corr_common.corrs, c->name, c) < 0) {
		fprintf(stderr, "calibtool: ailed to register %s procedure\n", c->name);
	}
}


void corr_done(void)
{
	calib_t *c;
	unsigned i;

	/* stop recalculation thread */
	// corr_common.run = false;
	// threadJoin(0);

	i = 0;
	c = (calib_t *)hmap_next(corr_common.corrs, &i);
	while (c != NULL) {
		if (c->proc.corr.done != NULL) {
			c->proc.corr.done();
		}
	}

	free(corr_common.sched);
}


int corr_init(const char *path)
{
	unsigned int timeVar, init, i;
	int retcode;
	bool err = false;
	calib_t *c;

	if (calib_read(path, corr_common.corrs) < 0) {
		return -EBADF;
	}

	/* count all time-variant corrections */
	timeVar = i = 0;
	c = (calib_t *)hmap_next(corr_common.corrs, &i);
	while (c != NULL) {
		if (c->proc.corr.delay != 0) {
			timeVar++;
		}
		c = (calib_t *)hmap_next(corr_common.corrs, &i);
	}

	/* init all calibrations that need it */
	init = i = 0;
	c = (calib_t *)hmap_next(corr_common.corrs, &i);
	while (c != NULL) {
		if (c->proc.corr.init != NULL) {
			retcode = c->proc.corr.init();
			if (retcode != 0) {
				err = true;
				break;
			}
		}
		init++;

		c = (calib_t *)hmap_next(corr_common.corrs, &i);
	}

	/* if error occured, deinit all corrections up to the failed one */
	if (err) {
		i = 0;
		while (init > 0) {
			c = (calib_t *)hmap_next(corr_common.corrs, &i);
			if (c->proc.calib.done != NULL) {
				c->proc.calib.done();
			}

			init--;
		}

		return retcode;
	}

	/* initialize scheduler table for time variant corrections */
	corr_common.sched = NULL;
	corr_common.schedSz = timeVar;
	if (timeVar > 0) {
		corr_common.sched = calloc(timeVar, sizeof(corr_sched_t));
		if (corr_common.sched == NULL) {
			return -ENOMEM;
		}
	}

	/* save all time-variant calibration procedures to newly created table */
	i = 0;
	while (timeVar > 0) {
		c = (calib_t *)hmap_next(corr_common.corrs, &i);
		if (c->proc.corr.delay != 0) {
			corr_common.sched[timeVar - 1] = (corr_sched_t) { .cal = c, .deadline = c->proc.corr.delay };
			timeVar--;
		}
	}

	return EOK;
}


static void corr_thread(void *arg)
{
	calib_t *corr;
	corr_sched_t *entry;
	time_t now;
	unsigned int i;

	/* run all time-variant corrections for the first time */
	for (i = 0; i < corr_common.schedSz; i++) {
		entry = &corr_common.sched[i];
		corr = entry->cal;

		/* run recalculation */
		corr->proc.corr.recalc();

		/* reschedule */
		gettime(&now, NULL);
		entry->deadline = now + corr->proc.corr.delay;
	}

	/* Simple EDF scheduler */
	if (corr_common.schedSz != 0) {
		while (corr_common.run) {
			/* find entry with earliest deadline */
			entry = &corr_common.sched[0];
			for (i = 1; i < corr_common.schedSz; i++) {
				if (corr_common.sched[i].deadline < entry->deadline) {
					entry = &corr_common.sched[i];
				}
			}
			corr = entry->cal;

			/* wait for the deadline */
			gettime(&now, NULL);
			if (entry->deadline > now) {
				usleep(entry->deadline - now);
			}

			/* perform and reschedule current correction */
			corr->proc.corr.recalc();
			entry->deadline = now + corr->proc.corr.delay;
		}
	}

	endthread();
}


int corr_run(void)
{
	return beginthread(corr_thread, 4, corr_common.stack, sizeof(corr_common.stack), NULL);
}


/* This constructor must run before calibration procedures constructors. It assures NULL-ability of hashmap */
__attribute__((constructor(101))) static void corr_premain(void)
{
	corr_common.corrs = hmap_init(CALIBS_SIZE);

	if (corr_common.corrs == NULL) {
		printf("sensc: hashmap allocation fail!");
		exit(EXIT_FAILURE);
	}
}


__attribute__((destructor(101))) static void corr_postmain(void)
{
	hmap_free(corr_common.corrs);
}
