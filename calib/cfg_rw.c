/*
 * Phoenix-Pilot
 *
 * calib config read/write separation for cleaner main.c
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

#include "procedures/calls.h"


static void cal_printCfg(FILE *file)
{
	int i, m;

	/* print mMotCalib */
	for (m = 0; m < 4; m++) {
		fprintf(file, "mMot pts %d", m);
		for (i = 0; i < CALIB_POINTS; i++) {
			fprintf(
				file, " %.2f %.2f %.2f %.2f",
				calibs_common.mMot.motCal[m][i].l,
				calibs_common.mMot.motCal[m][i].x,
				calibs_common.mMot.motCal[m][i].y,
				calibs_common.mMot.motCal[m][i].z);
		}
		fprintf(file, "\n");
	}
}

/* Config file read into `calib_common` structure */
int cal_calibsRead(const char *filepath)
{
	FILE *fd = fopen(filepath, "r");

	if (fd == NULL) {
		printf("%s not found\n", filepath);
		return 0;
	}

	fclose(fd);

	return 0;
}

/* Overwriting config file with values from `calib_common` structure */
int cal_calibsWrite(const char *filepath)
{
	FILE *fd = fopen(filepath, "w");

	if (fd == NULL) {
		printf("Failed to open config file '%s'! Printing config on screen\n", filepath);
		cal_printCfg(stdout);
		return -1;
	}
	cal_printCfg(fd);
	fclose(fd);

	return 0;
}
