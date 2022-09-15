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
	calib_data_t params;
} magiron_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *magiron_calibStructGet(void)
{
	magiron_common.params.type = typeMagiron;
	return &magiron_common.params;
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


static int cal_magironWrite(FILE *file)
{
	/* Printing hard iron calibration parameters */
	magiron_printIron(file, CHAR_HARDIRON, &magiron_common.params.params.magiron.hardCal);

	/* Printing soft iron calibration parameters */
	magiron_printIron(file, CHAR_SOFTIRON, &magiron_common.params.params.magiron.softCal);

	return 0;
}


static const char *cal_magironHelp(void)
{
	return "Magnetometer calibration against soft/hard iron interference.\n";
}


static int cal_magironDone(void)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


static int cal_magironRun(void)
{
	printf("This calibration procedure is not implemented and it returns precalculated values!\n Press enter to continue...\n");
	getchar();
	fflush(stdin);

	return EOK;
}


static int cal_magironInit(int argc, const char **argv)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


__attribute__((constructor(102))) static void cal_magironRegister(void)
{
	static calib_ops_t cal = {
		.name = MAGIRON_TAG,
		.init = cal_magironInit,
		.run = cal_magironRun,
		.done = cal_magironDone,
		.write = cal_magironWrite,
		.help = cal_magironHelp,
		.dataGet = magiron_calibStructGet
	};

	calib_register(&cal);
}
