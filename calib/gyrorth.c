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

#include <matrix.h>

#include "calibtool.h"

struct {
	calib_data_t data;
} gyrorth_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *gyrorth_dataGet(void)
{
	return &gyrorth_common.data;
}


static const char *gyrorth_help(void)
{
	return "Gyroscope nonorthogonality calibration\n";
}


static int gyrorth_write(FILE *file)
{
	unsigned int row, col;

	for (row = 0; row < ACC_OFFSET_ROWSPAN; row++) {
		for (col = 0; col < ACC_OFFSET_COLSPAN; col++) {
			fprintf(file, "%c%u%u %f\n", ACC_CHAR_OFFSET, row, col, MATRIX_DATA(&gyrorth_common.data.params.gyrorth.offset, row, col));
		}
	}

	for (row = 0; row < ACC_ORTHO_ROWSPAN; row++) {
		for (col = 0; col < ACC_ORTHO_COLSPAN; col++) {
			fprintf(file, "%c%u%u %f\n", ACC_CHAR_ORTHO, row, col, MATRIX_DATA(&gyrorth_common.data.params.gyrorth.ortho, row, col));
		}
	}
	return EOK;
}


static int gyrorth_done(void)
{
	return EOK;
}


int gyrorth_run(void)
{
	printf("This calibration is not yet implemented!\n");
	printf("This calibration returns default parameters if there are none in calibration file\n");
	printf("Calibration done!\n");

	return EOK;
}


static int gyrorth_init(int argc, const char **argv)
{
	return EOK;
}


__attribute__((constructor(102))) static void gyrorth_register(void)
{
	static calib_ops_t cal = {
		.name = GYRORTH_TAG,
		.init = gyrorth_init,
		.run = gyrorth_run,
		.done = gyrorth_done,
		.write = gyrorth_write,
		.help = gyrorth_help,
		.dataGet = gyrorth_dataGet
	};

	calib_register(&cal);

	gyrorth_common.data.type = typeGyrorth;
}
