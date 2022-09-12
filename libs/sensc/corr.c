/*
 * Phoenix-Pilot
 * 
 * sensorhub client corrections module
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
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include <calib.h>
#include <hmap.h>

#include "corr.h"


typedef struct _corr_entry_t {
	calib_t *corr;
	time_t trigTime;
} corr_entry_t;


struct {
	hmap_t *corrs;

	corr_entry_t *dynCorrs;
	unsigned int dynCorrsLen;
	volatile bool run;
} corr_common;



static void corr_thread(void *arg)
{
	corr_entry_t *entries, *nextEntry;
	time_t now;
	unsigned int i, entriesSz;

	/* local variables for better readibility */
	entries = corr_common.dynCorrs;
	entriesSz = corr_common.dynCorrsLen;

	/* run all corrections for the first time */
	gettime(&now, NULL);
	for (i = 0; i < entriesSz; i++) {
		entries[i].corr->corrRecalc();
		entries[i].trigTime = now + entries[i].corr->delay;
	}

	/* dont run loop if no dynamic corrections available */
	if (entriesSz == 0) {
		while (corr_common.run)
		{
			/* Find the most urgent correction racalculation */
			nextEntry = &entries[0];
			for (i = 1; i < entriesSz; i++) {
				if (entries[i].trigTime <= nextEntry->trigTime) {
					nextEntry = &entries[i];
				}
			}

			/* Sleep till the most urgent recalculation */
			gettime(&now, NULL);
			if (now < nextEntry->trigTime) {
				usleep(nextEntry->trigTime - now);
			}

			/* Perform and reschedule the most urgent recalculation */
			nextEntry->corr->corrRecalc();
			nextEntry->trigTime = now + nextEntry->corr->delay;
		}
	}

	endthread();
}


/*
* Perform magnetometer corrections in the following order:
* 1) magiron - hard/soft iron correction
* 2) magmot - motor interference
*/
int corr_mag(sensor_event_t *magEvt)
{
	static calib_t *magmotCorr = NULL;
	static calib_t *magironCorr = NULL;

	/* load correction procedures necessary for magnetometer */
	if (magmotCorr == NULL) {
		magmotCorr = hmap_get(corr_common.corrs, "magiron");
	}
	if (magironCorr == NULL) {
		magmotCorr = hmap_get(corr_common.corrs, "magmot");
	}

	/* use calibration procedures for magnetometer */
	if (magironCorr != NULL) {
		magironCorr->corrDo(magEvt);
	}
	if (magmotCorr != NULL) {
		magmotCorr->corrDo(magEvt);
	}

	return 0;
}


void corr_done(void)
{
	calib_t *corr;
	unsigned int i;

	/* stop the thread logic here */
	corr_common.run = false;
	threadJoin(0);

	i = 0;
	corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	while (corr != NULL) {
		if (corr->corrDone != NULL) {
			corr->corrDone();
		}
		corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	}

	free(corr_common.dynCorrs);
}


int corr_init(void)
{
	calib_t *corr;
	unsigned int inited, i, dynCorrs;
	bool err = false;

	corr_common.corrs = calib_hashmapGet();

	if (calib_read(CALIB_FILE, corr_common.corrs) != 0) {
		fprintf(stderr, "corr: failed to read calibration file\n");
		return -1;
	}

	/* find all time-variant corrections */
	i = dynCorrs = 0;
	corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	while (corr != NULL) {
		if (corr->delay != 0) {
			dynCorrs++;
		}
		corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	}

	/* Create array for all time-variant corrections */
	corr_common.dynCorrsLen = dynCorrs;
	corr_common.dynCorrs = NULL;
	if (dynCorrs > 0) {
		corr_common.dynCorrs = calloc(dynCorrs, sizeof(corr_entry_t));
		if (corr_common.dynCorrs == NULL) {
			fprintf(stderr, "corr: failed to allocate correcton scheduler entries/n");
			return -1;
		}
	}

	/* write all time-variant corrections to the scheduler array */
	corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	i = dynCorrs = 0;
	while (corr != NULL) {
		if (corr->delay != 0) {
			corr_common.dynCorrs[dynCorrs].corr = corr;
			corr_common.dynCorrs[dynCorrs].trigTime = 0;
		}
		corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	}

	/* Initialize all corrections that need it */
	i = inited = 0;
	corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	while (corr != NULL && !err) {
		if (corr->corrInit != NULL) {
			if (corr->corrInit() != 0) {
				err = true;
				fprintf(stderr, "corr: failed to init %s correction procedure\n", corr->name);
				break;
			}
		}
		inited++;
		corr = (calib_t*)hmap_next(corr_common.corrs, &i);
	}

	/* in case of error deinitialize all corrections up to the failed one */
	if (err) {
		i = 0;
		while (inited > 0) {
			corr = (calib_t*)hmap_next(corr_common.corrs, &i);
			if (corr->corrDone != NULL) {
				corr->corrDone();
			}
			inited--;
		}

		free(corr_common.dynCorrs);
		return -1;
	}

	return 0;
}
