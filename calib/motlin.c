/*
 * Phoenix-Pilot
 *
 * Drone motors linear compensation procedure
 * Calibration of inequalities between engines PWMs that level the drone
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>

#include "calibtool.h"

struct {
	calib_data_t data;
} motlin_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *motlin_dataGet(void)
{
	return &motlin_common.data;
}


static const char *motlin_help(void)
{
	return "Linear calibration of motors\n";
}


static int motlin_write(FILE *file)
{
	int i;

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		fprintf(file, "ml%da %f\n", i, motlin_common.data.params.motlin.motorEq[i][0]);
		fprintf(file, "ml%db %f\n", i, motlin_common.data.params.motlin.motorEq[i][1]);
	}

	return EOK;
}


static int motlin_done(void)
{
	return EOK;
}


int motlin_run(void)
{
	printf("This calibration returns default parameters if there are none in calibration file\n");
	printf("Calibration done!");

	return EOK;
}


static int motlin_init(int argc, const char **argv)
{
	return EOK;
}


__attribute__((constructor(102))) static void motlin_register(void)
{
	static calib_ops_t cal = {
		.name = MOTLIN_TAG,
		.init = motlin_init,
		.run = motlin_run,
		.done = motlin_done,
		.write = motlin_write,
		.help = motlin_help,
		.dataGet = motlin_dataGet
	};

	calib_register(&cal);

	motlin_common.data.type = typeMotlin;
}
