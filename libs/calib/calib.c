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


static inline void calib_magmotDefaults(calib_t *cal)
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
	if (calib_file2tag(file, MAGMOT_TAG) != 0) {
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


/* returns pointer do data slot named as 'paramName' */
static float *magiron_paramSlot(const char *paramName, calib_t *cal)
{
	matrix_t *mat;
	unsigned int row, col;

	if (strlen(paramName) != 3) {
		return NULL;
	}

	/* variable casting for MISRA compliance */
	row = (uint8_t)(paramName[1] - '0'); /* convert character to unsigned int digit */
	col = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */


	/* matrix type get through character check */
	switch (paramName[0]) {
		case CHAR_SOFTIRON:
			if (row > (unsigned int)'9' || col > (unsigned int)'9') {
				return NULL;
			}
			mat = &cal->params.magiron.softCal;
			break;

		case CHAR_HARDIRON:
			if (row > (unsigned int)'3' || col > (unsigned int)'3') {
				return NULL;
			}
			mat = &cal->params.magiron.hardCal;
			break;

		default:
			return NULL;
	}

	/* matrix boundary checks performed by matrix_at() */
	return matrix_at(mat, row, col);
}


static inline void calib_magironDefaults(calib_t *cal)
{
	/* Creating constant aliases of matrices */
	const matrix_t hardCal = cal->params.magiron.hardCal;
	const matrix_t softCal = cal->params.magiron.softCal;

	/* hard iron vector matrix */
	*matrix_at(&hardCal, 0, 0) = 42.47503636;
	*matrix_at(&hardCal, 1, 0) = 1084.20661751;
	*matrix_at(&hardCal, 2, 0) = -111.58247011;

	/* soft iron matrix */
	*matrix_at(&softCal, 0, 0) = 0.9409439;
	*matrix_at(&softCal, 0, 1) = 0.09766692;
	*matrix_at(&softCal, 0, 2) = -0.01307758;
	*matrix_at(&softCal, 1, 0) = 0.09766692;
	*matrix_at(&softCal, 1, 1) = 1.01364504;
	*matrix_at(&softCal, 1, 2) = -0.01144832;
	*matrix_at(&softCal, 2, 0) = -0.01307758;
	*matrix_at(&softCal, 2, 1) = -0.01144832;
	*matrix_at(&softCal, 2, 2) = 1.0593312;
}


static int calib_magironRead(FILE *file, calib_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float val = 0, *valuePtr;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	/* Scroll to 'magiron tag */
	if (calib_file2tag(file, MAGIRON_TAG) != 0) {
		return -1;
	}

	/* allocate matrix buffers */
	if (matrix_bufAlloc(&cal->params.magiron.hardCal, HARDCAL_ROWSPAN, HARDCAL_COLSPAN) != 0) {
		return -1;
	}
	if (matrix_bufAlloc(&cal->params.magiron.softCal, SOFTCAL_ROWSPAN, SOFTCAL_COLSPAN) != 0) {
		matrix_bufFree(&cal->params.magiron.hardCal);
		return -1;
	}

	calib_magironDefaults(cal);

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &val) != 0) {
		valuePtr = magiron_paramSlot(name, cal);
		if (valuePtr == NULL) {
			break;
		}
		*valuePtr = val;
		params++;
	}
	free(line);

	if (params != MAGIRON_PARAMS) {
		calib_magmotDefaults(cal);
		fprintf(stderr, "Failed to read `magiron` calibration. Going default.\n");
	}

	return 0;
}


void calib_free(calib_t *cal)
{
	switch (cal->type) {
		case typeMagiron:
			matrix_bufFree(&cal->params.magiron.hardCal);
			matrix_bufFree(&cal->params.magiron.softCal);
			break;

		case typeMagmot:
		default:
			return;
	}
}


int calib_readFile(const char *path, calibType_t type, calib_t *cal)
{
	FILE *file;
	int ret;

	if (path == NULL || cal == NULL) {
		return -1;
	}

	file = fopen(path, "r");
	if (file == NULL) {
		fprintf(stderr, "calib: cannot open calibration file '%s'\n", path);
		return -1;
	}

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
