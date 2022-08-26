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
	vec_t *vec;

	/* mMot specific */
	int motor;
	char abc, xyz;

	if (strstr(valName, "mMot") != NULL) {
		if (strlen(valName) < 10) {
			return -1;
		}

		/* 3 specifiers to be found: motor, a/b/c parameter, x/y/z parameter: mMot_m0_a_x */
		motor = valName[6] - '0'; /* to get the motor digit as int */
		abc = valName[8];
		xyz = valName[10];

		switch (abc) {
			case 'a':
				vec = &calibs_common.mMot.motCal[motor].a;
				break;
			case 'b':
				vec = &calibs_common.mMot.motCal[motor].b;
				break;
			case 'c':
				vec = &calibs_common.mMot.motCal[motor].c;
				break;
			default:
				return -1;
				break;
		}

		switch (xyz) {
			case 'x':
				vec->x = val;
				break;
			case 'y':
				vec->y = val;
				break;
			case 'z':
				vec->z = val;
				break;
			default:
				return -1;
				break;
		}

	}

	return 0;
}


static void cal_defCfgSet(void)
{
	int i;
	for (i = 0; i < 4; i++) {
		vec_times(&calibs_common.mMot.motCal[i].a, 0);
		vec_times(&calibs_common.mMot.motCal[i].b, 0);
		vec_times(&calibs_common.mMot.motCal[i].c, 0);
	}
}


static void cal_printCfg(FILE *file)
{
	int i;

	for (i = 0; i < 4; i++) {
		fprintf(file, "mMot_m%d_a_x %f\n", i, calibs_common.mMot.motCal[i].a.x);
		fprintf(file, "mMot_m%d_a_y %f\n", i, calibs_common.mMot.motCal[i].a.y);
		fprintf(file, "mMot_m%d_a_z %f\n", i, calibs_common.mMot.motCal[i].a.z);
		fprintf(file, "mMot_m%d_b_x %f\n", i, calibs_common.mMot.motCal[i].b.x);
		fprintf(file, "mMot_m%d_b_y %f\n", i, calibs_common.mMot.motCal[i].b.y);
		fprintf(file, "mMot_m%d_b_z %f\n", i, calibs_common.mMot.motCal[i].b.z);
		fprintf(file, "mMot_m%d_c_x %f\n", i, calibs_common.mMot.motCal[i].c.x);
		fprintf(file, "mMot_m%d_c_y %f\n", i, calibs_common.mMot.motCal[i].c.y);
		fprintf(file, "mMot_m%d_c_z %f\n", i, calibs_common.mMot.motCal[i].c.z);
	}
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

		if (cal_mMotCfgRead(p, val) < 0) {
			return -1;
		}
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
