/*
 * Phoenix-Pilot
 *
 * Drone calibration module.
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

#include <calibcore.h>
#include <hmap.h>


struct {
	hmap_t *calibs;
} calib_common;


/* Register new calib_t procedure. */
void calib_register(calib_t *c)
{
	if (hmap_insert(calib_common.calibs, c->name, c) < 0) {
		fprintf(stderr, "calibtool: ailed to register %s procedure\n", c->name);
	}
}


hmap_t *calib_hashmapGet(void)
{
	return calib_common.calibs;
}


/* This constructor must run before calibration procedures constructors. It assures NULL-ability of hashmap */
__attribute__((constructor(101))) static void calib_premain(void)
{
	calib_common.calibs = hmap_init(CALIBS_SIZE);

	if (calib_common.calibs == NULL) {
		printf("sensc: hashmap allocation fail!");
		exit(EXIT_FAILURE);
	}
}


__attribute__((destructor(101))) static void calib_postmain(void)
{
	hmap_free(calib_common.calibs);
}
