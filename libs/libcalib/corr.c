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
#include <string.h>
#include <errno.h>

#include "libcalib.h"
#include "hmap.h"


struct {
	hmap_t *corrs;
} corr_common;


/* Register new calib_t procedure. */
void common_register(calib_t *c)
{
	if (hmap_insert(corr_common.corrs, c->name, c) < 0) {
		fprintf(stderr, "calibtool: ailed to register %s procedure\n", c->name);
	}
}


hmap_t *corr_hashmapGet(void)
{
	return corr_common.corrs;
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
