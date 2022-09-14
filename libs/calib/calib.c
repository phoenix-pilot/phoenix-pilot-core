/*
 * Phoenix-Pilot
 *
 * Calibration library
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
#include <stdlib.h>

#include "calib.h"

/* Scroll 'file' until 'tag' is reached. Returns 0 on success */
static int calib_file2tag(FILE *file, const char *tag)
{
	char *line = NULL;
	size_t lineSz;
	int ret = -1;

	while (getline(&line, &lineSz, file) >= 0) {
		if (line[0] == '@') {
			if (strcmp(&line[1], tag) == 0) {
				/* matching tag */
				ret = 0;
				break;
			}
		}
	}

	free(line);
	return ret;
}


/* Getline wrapper that reads line to buffer pointed by 'bufptr' and finds parameter 'name' and its value in that buffer */
static int calib_getline(char **bufptr, size_t *bufSz, FILE *file, char **name, float *val)
{
	char *head, *value;

	if (getline(bufptr, bufSz, file) > 0) {
		head = strtok(*bufptr, " \n");
		value = strtok(NULL, " \n");

		if (head == NULL || value == NULL) {
			return -1;
		}

		*name = head;
		*val = atof(value);

		return 0;
	}

	return -1;
}


/* Returns pointer to correct parameter variable given name `paramName` for magmot calibration */
static float *calib_magmotSlot(const char *paramName, calib_t *cal)
{
	unsigned int motor, axis, param;

	if (strlen(paramName) != 4) {
		return NULL;
	}

	/* variable casting for MISRA compliance */
	motor = (uint8_t)(paramName[1] - '0'); /* get motor id */
	axis = (uint8_t)(paramName[2] - 'x');  /* get x/y/z index, knowing that x/y/z are consecutive in ASCII */
	axis = (uint8_t)(paramName[3] - 'a');  /* get a/b/c index, knowing that a/b/c are consecutive in ASCII */

	if (motor >= NUM_OF_MOTORS || axis >= 3 || param >= 3) {
		return NULL;
	}

	return &cal->params.magmot.motorEq[motor][axis][param];
}


static void calib_magmotDefaults(calib_t *cal)
{
	/* set all parameters to 0 */
	memset(cal->params.magmot.motorEq, 0, sizeof(cal->params.magmot.motorEq));
}


static int calib_magmotRead(FILE *file, calib_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float value = 0;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	/* scroll to the `magmot` tag */
	if (calib_file2tag(file, "magmot") != 0) {
		return -1;
	}

	calib_magmotDefaults(cal);

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &value) != 0) {
		*calib_magmotSlot(name, cal) = value;
		params++;
	}
	free(line);

	if (params != MAGMOT_PARAMS) {
		calib_magmotDefaults(cal);
		return -1;
	}

	return 0;
}


static int calib_magironRead(FILE *file, calib_t *cal)
{
	if (calib_file2tag(file, "magiron") != 0) {
		return -1;
	}

	return 0;
}


/* read calibration file pointed by 'path' searching for calibration named `tag` and saving its content to 'cal' */
int calib_readFile(const char *path, calibType_t type, calib_t *cal)
{
	FILE *file;
	int ret;

	file = fopen(path, "r");
	if (file == NULL) {
		fprintf(stderr, "calib: cannot open calibration file '%s'\n", path);
		return -1;
	}

	ret = -1;
	switch (type) {
		case typeMagmot:
			ret = calib_magmotRead(file, cal);
			break;

		case typeMagiron:
			ret = calib_magironRead(file, cal);
			break;

		default:
			ret = -1;
	}

	fclose(file);

	return ret;
}
