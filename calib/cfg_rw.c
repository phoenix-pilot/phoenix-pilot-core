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


static int cal_mMotCfgRead(const char * valName, float val)
{
	if (strcmp(valName, "mMot_ax")) {
		calibs_common.mMot.ax = val;
	}
	else if (strcmp(valName, "mMot_ay")) {
		calibs_common.mMot.ay = val;
	}
	else if (strcmp(valName, "mMot_az")) {
		calibs_common.mMot.az = val;
	}
	else if (strcmp(valName, "mMot_bx")) {
		calibs_common.mMot.bx = val;
	}
	else if (strcmp(valName, "mMot_by")) {
		calibs_common.mMot.by = val;
	}
	else if (strcmp(valName, "mMot_bz")) {
		calibs_common.mMot.bz = val;
	}

	return 0;
}


static void cal_defCfgSet(void)
{
	calibs_common.mMot.ax = calibs_common.mMot.ay = calibs_common.mMot.az = 0;
	calibs_common.mMot.bx = calibs_common.mMot.by = calibs_common.mMot.bz = 0;

	calibs_common.aRot.dummyVal = 0;
	calibs_common.mStatic.dummyVal = 0;
}


static void cal_printCfg(FILE *file)
{
	fprintf(file, "mMot_ax %f\n", calibs_common.mMot.ax);
	fprintf(file, "mMot_ay %f\n", calibs_common.mMot.ay);
	fprintf(file, "mMot_az %f\n", calibs_common.mMot.az);
	fprintf(file, "mMot_bx %f\n", calibs_common.mMot.bx);
	fprintf(file, "mMot_by %f\n", calibs_common.mMot.by);
	fprintf(file, "mMot_bz %f\n", calibs_common.mMot.bz);
}


/* Config file read into `calib_common` structure */
int cal_calibsRead(const char *filepath)
{
	char buf[32], *p, *v;
	float val;
	FILE *fd = fopen(filepath, "r");

	cal_defCfgSet();

	if (fd == NULL) {
		printf("%s not found\n", filepath);
		return 0;
	}

	while (fgets(buf, sizeof(buf), fd)) {
		p = strtok(buf, " ");
		v = strtok(NULL, " ");
		val = atof(v);

		cal_mMotCfgRead(p, val);
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
