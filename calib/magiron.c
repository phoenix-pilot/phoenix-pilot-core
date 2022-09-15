/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against hard/soft iron interference
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <matrix.h>
#include <vec.h>
#include <calib.h>

#include "calibtool.h"


struct {
	calib_data_t data;
} magiron_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *magiron_dataGet(void)
{
	return &magiron_common.data;
}


/* Prints to `file` data from `mat` as calibtype `type` */
static void magiron_printIron(FILE *file, char type, matrix_t *mat)
{
	unsigned int rows, cols, r, c;

	cols = matrix_colsGet(mat);
	rows = matrix_rowsGet(mat);

	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			fprintf(file, "%c%i%i %f\n", type, r, c, *matrix_at(mat, r, c));
		}
	}
}


static int magiron_write(FILE *file)
{
	/* Printing hard iron calibration parameters */
	magiron_printIron(file, CHAR_HARDIRON, &magiron_common.data.params.magiron.hardCal);

	/* Printing soft iron calibration parameters */
	magiron_printIron(file, CHAR_SOFTIRON, &magiron_common.data.params.magiron.softCal);

	return 0;
}


static const char *magiron_help(void)
{
	return "Magnetometer calibration against soft/hard iron interference.\n";
}


static int magiron_done(void)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


static int magiron_run(void)
{
	printf("This calibration procedure is not implemented and it returns precalculated values!\n Press enter to continue...\n");
	getchar();
	fflush(stdin);

	return EOK;
}


static int magiron_init(int argc, const char **argv)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


__attribute__((constructor(102))) static void magiron_register(void)
{
	static calib_ops_t cal = {
		.name = MAGIRON_TAG,
		.init = magiron_init,
		.run = magiron_run,
		.done = magiron_done,
		.write = magiron_write,
		.help = magiron_help,
		.dataGet = magiron_dataGet
	};

	calib_register(&cal);

	magiron_common.data.type = typeMagiron;
}
