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
#include <stdint.h>

#include "calib.h"

/* Scroll 'file' until 'tag' is reached. Returns 0 on success */
static int calib_file2tag(FILE *file, const char *tag)
{
	char *head, *line = NULL;
	size_t lineSz;
	int ret = -1;

	while (getline(&line, &lineSz, file) >= 0) {
		if (line[0] == '@') {
			head = strtok(line, " \n");
			if (strcmp(&(head[1]), tag) == 0) {
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

	if (getline(bufptr, bufSz, file) >= 0) {
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


/* Returns 0 if new value `val` was successfully saved to `cal` as `paramName` parameter. -1 otherwise */
static int calib_magmotEnter(const char *paramName, calib_data_t *cal, float val)
{
	unsigned int motor, axis, param;

	if (strlen(paramName) != 4) {
		return -1;
	}

	motor = (uint8_t)(paramName[1] - '0'); /* get motor id */
	axis = (uint8_t)(paramName[2] - 'x');  /* get x/y/z index, knowing that x/y/z are consecutive in ASCII */
	param = (uint8_t)(paramName[3] - 'a'); /* get a/b/c index, knowing that a/b/c are consecutive in ASCII */

	if (motor >= NUM_OF_MOTORS || axis >= 3 || param >= 3) {
		return -1;
	}

	cal->params.magmot.motorEq[motor][axis][param] = val;

	return 0;
}


static inline void calib_magmotDefaults(calib_data_t *cal)
{
	/* set all parameters to 0 */
	memset(cal->params.magmot.motorEq, 0, sizeof(cal->params.magmot.motorEq));
}


static int calib_magmotRead(FILE *file, calib_data_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float value = 0;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	calib_magmotDefaults(cal);

	if (file == NULL) {
		fprintf(stderr, "No calibration file. '%s' going default.\n", MAGMOT_TAG);
		return 0;
	}

	/* scroll to the `magmot` tag */
	if (calib_file2tag(file, MAGMOT_TAG) != 0) {
		fprintf(stderr, "Calibration not done yet. '%s' going default.\n", MAGMOT_TAG);
		return 0;
	}

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &value) == 0) {
		if (calib_magmotEnter(name, cal, value) != 0) {
			break;
		}
		params++;
	}
	free(line);

	if (params != MAGMOT_PARAMS) {
		calib_magmotDefaults(cal);
		fprintf(stderr, "Failed to read `%s` calibration. Going default.\n", MAGMOT_TAG);
	}

	return 0;
}


/* Returns 0 if new value `val` was successfully saved to `cal` as `paramName` parameter. -1 otherwise */
static int calib_magironEnter(const char *paramName, calib_data_t *cal, float val)
{
	matrix_t *mat;
	unsigned int row, col;
	float *slot;

	if (strlen(paramName) != 3) {
		return -1;
	}

	row = (uint8_t)(paramName[1] - '0'); /* convert character to unsigned int digit */
	col = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */


	/* matrix type get through character check */
	switch (paramName[0]) {
		case CHAR_SOFTIRON:
			if (row > 9 || col > 9) {
				return -1;
			}
			mat = &cal->params.magiron.softCal;
			break;

		case CHAR_HARDIRON:
			if (row > 3 || col > 3) {
				return -1;
			}
			mat = &cal->params.magiron.hardCal;
			break;

		default:
			return -1;
	}

	/* matrix boundary checks performed by matrix_at() */
	slot = matrix_at(mat, row, col);

	if (slot == NULL) {
		return -1;
	}

	*slot = val;

	return 0;
}


static inline void calib_magironDefaults(calib_data_t *cal)
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


static int calib_magironRead(FILE *file, calib_data_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float val = 0;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	/* allocate matrix buffers */
	if (matrix_bufAlloc(&cal->params.magiron.hardCal, HARDCAL_ROWSPAN, HARDCAL_COLSPAN) != 0) {
		return -1;
	}
	if (matrix_bufAlloc(&cal->params.magiron.softCal, SOFTCAL_ROWSPAN, SOFTCAL_COLSPAN) != 0) {
		matrix_bufFree(&cal->params.magiron.hardCal);
		return -1;
	}

	calib_magironDefaults(cal);

	if (file == NULL) {
		fprintf(stderr, "No calibration file. '%s' going default.\n", MAGIRON_TAG);
		return 0;
	}

	/* Scroll to 'magiron' tag */
	if (calib_file2tag(file, MAGIRON_TAG) != 0) {
		fprintf(stderr, "Calibration not done yet. '%s' going default.\n", MAGIRON_TAG);
		return 0;
	}

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &val) == 0) {
		if (calib_magironEnter(name, cal, val) != 0) {
			break;
		}
		params++;
	}
	free(line);

	if (params != MAGIRON_PARAMS) {
		calib_magmotDefaults(cal);
		fprintf(stderr, "Failed to read `%s` calibration. Going default.\n", MAGIRON_TAG);
	}

	return 0;
}


/* Returns 0 if new value `val` was successfully saved to `cal` as `paramName` parameter. -1 otherwise */
static int calib_accorthEnter(const char *paramName, calib_data_t *cal, float val)
{
	matrix_t *mat = NULL;
	unsigned int row, col, axis;
	char swapVal;
	float *slot;

	if (strlen(paramName) != 3) {
		return -1;
	}

	row = (uint8_t)(paramName[1] - '0'); /* convert character to unsigned int digit */
	col = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */

	swapVal = paramName[1];
	axis = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */

	/* matrix type get through character check */
	switch (paramName[0]) {
		case ACC_CHAR_ORTHO:
			if (row > 9 || col > 9) {
				return -1;
			}
			mat = &cal->params.accorth.ortho;
			break;

		case ACC_CHAR_OFFSET:
			if (row > 3 || col > 3) {
				return -1;
			}
			mat = &cal->params.accorth.offset;
			break;

		case ACC_CHAR_QUAT:
			if (col > 0) {
				return -1;
			}
			/* Saving quaternion */
			switch (row) {
				case 0:
					cal->params.accorth.frameQ.a = val;
					break;
				case 1:
					cal->params.accorth.frameQ.i = val;
					break;
				case 2:
					cal->params.accorth.frameQ.j = val;
					break;
				case 3:
					cal->params.accorth.frameQ.k = val;
					break;
				default:
					return -1;
			}
			return 0;

		case ACC_CHAR_SWAP:
			switch (swapVal) {
				case ACC_CHAR_SWAP_ORDR:
					cal->params.accorth.swapOrder = val;
					break;

				case ACC_CHAR_SWAP_SIGN:
					if ((val != 0 && val != 1) || axis > 2 || axis < 0) {
						fprintf(stderr, "accorth invalid swap sign\n");
						return -1;
					}
					cal->params.accorth.axisInv[axis] = val;
					break;

				default:
					return -1;
			}
			return 0;

		default:
			return -1;
	}

	/* matrix boundary checks performed by matrix_at() */
	slot = matrix_at(mat, row, col);

	if (slot == NULL) {
		return -1;
	}

	*slot = val;

	return 0;
}


static inline void calib_accorthDefaults(calib_data_t *cal)
{
	/* Creating constant aliases of matrices */
	const matrix_t offset = cal->params.accorth.offset;
	const matrix_t ortho = cal->params.accorth.ortho;

	/* ellipsoid center offset matrix */
	*matrix_at(&offset, 0, 0) = 0;
	*matrix_at(&offset, 1, 0) = 0;
	*matrix_at(&offset, 2, 0) = 0;

	/* ellipsoid deformation matrix */
	*matrix_at(&ortho, 0, 0) = 1;
	*matrix_at(&ortho, 0, 1) = 0;
	*matrix_at(&ortho, 0, 2) = 0;
	*matrix_at(&ortho, 1, 0) = 0;
	*matrix_at(&ortho, 1, 1) = 1;
	*matrix_at(&ortho, 1, 2) = 0;
	*matrix_at(&ortho, 2, 0) = 0;
	*matrix_at(&ortho, 2, 1) = 0;
	*matrix_at(&ortho, 2, 2) = 1;

	/* Accelerometer frame of reference rotatation */
	cal->params.accorth.frameQ.a = 1;
	cal->params.accorth.frameQ.i = 0;
	cal->params.accorth.frameQ.j = 0;
	cal->params.accorth.frameQ.k = 0;

	/* By default no swapping is performed */
	cal->params.accorth.axisInv[0] = 0;
	cal->params.accorth.axisInv[1] = 0;
	cal->params.accorth.axisInv[2] = 0;
	cal->params.accorth.swapOrder = 123;
}


static int calib_accorthRead(FILE *file, calib_data_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float val = 0, diff;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	/* allocate matrix buffers */
	if (matrix_bufAlloc(&cal->params.accorth.offset, ACC_OFFSET_ROWSPAN, ACC_OFFSET_COLSPAN) != 0) {
		return -1;
	}
	if (matrix_bufAlloc(&cal->params.accorth.ortho, ACC_ORTHO_ROWSPAN, ACC_ORTHO_COLSPAN) != 0) {
		matrix_bufFree(&cal->params.accorth.offset);
		return -1;
	}

	calib_accorthDefaults(cal);

	if (file == NULL) {
		fprintf(stderr, "No calibration file. '%s' going default.\n", ACCORTH_TAG);
		return 0;
	}

	/* Scroll to 'accortho' tag */
	if (calib_file2tag(file, ACCORTH_TAG) != 0) {
		fprintf(stderr, "Calibration not done yet. '%s' going default.\n", ACCORTH_TAG);
		return 0;
	}

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &val) == 0) {
		if (calib_accorthEnter(name, cal, val) != 0) {
			break;
		}
		params++;
	}
	free(line);

	if (params != ACCORTH_PARAMS) {
		calib_accorthDefaults(cal);
		fprintf(stderr, "Failed to read `%s` calibration. Going default.\n", ACCORTH_TAG);
	}

	diff = 1.f - quat_len(&cal->params.accorth.frameQ);
	if (diff > ACC_QUAT_ERR || diff < -ACC_QUAT_ERR) {
		fprintf(stderr, "calib %s: wrong quaternion norm of %f\n", ACCORTH_TAG, 1.f - diff);
		matrix_bufFree(&cal->params.accorth.offset);
		matrix_bufFree(&cal->params.accorth.ortho);
		return -1;
	}

	if (MATRIX_DATA(&cal->params.accorth.ortho, 0, 0) < 0 || MATRIX_DATA(&cal->params.accorth.ortho, 1, 1) < 0 || MATRIX_DATA(&cal->params.accorth.ortho, 2, 2) < 0) {
		fprintf(stderr, "calib %s: invalid S matrix\n", ACCORTH_TAG);
		matrix_bufFree(&cal->params.accorth.offset);
		matrix_bufFree(&cal->params.accorth.ortho);
		return -1;
	}

	return 0;
}


static void calib_motlinDefaults(calib_data_t *cal)
{
	cal->params.motlin.motorEq[0][0] = 0.968600;
	cal->params.motlin.motorEq[0][1] = 0.034796;

	cal->params.motlin.motorEq[1][0] = 1.031400;
	cal->params.motlin.motorEq[1][1] = 0.085204;

	cal->params.motlin.motorEq[2][0] = 1.003427;
	cal->params.motlin.motorEq[2][1] = 0.142546;

	cal->params.motlin.motorEq[3][0] = 0.996573;
	cal->params.motlin.motorEq[3][1] = 0.137454;
}


static int calib_motlinEnter(const char *paramName, calib_data_t *cal, float val)
{
	unsigned int motor, param;

	if (strlen(paramName) != 4) {
		return -1;
	}

	motor = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */
	param = (uint8_t)(paramName[3] - 'a'); /* convert character to unsigned int digit, expected range [a, b] -> [0, 1] */

	if (param > 1 || motor >= NUM_OF_MOTORS) {
		return -1;
	}

	cal->params.motlin.motorEq[motor][param] = val;

	return 0;
}


static int calib_motlinRead(FILE *file, calib_data_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float val = 0;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	calib_motlinDefaults(cal);

	if (file == NULL) {
		fprintf(stderr, "No calibration file. '%s' going default.\n", MOTLIN_TAG);
		return 0;
	}

	/* Scroll to 'motlin' tag */
	if (calib_file2tag(file, MOTLIN_TAG) != 0) {
		fprintf(stderr, "Calibration not done yet. '%s' going default.\n", MOTLIN_TAG);
		return 0;
	}

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &val) == 0) {
		if (calib_motlinEnter(name, cal, val) != 0) {
			break;
		}
		params++;
	}
	free(line);

	if (params != MOTLIN_PARAMS) {
		calib_magmotDefaults(cal);
		fprintf(stderr, "Failed to read `%s` calibration. Going default.\n", MOTLIN_TAG);
	}

	return 0;
}


void calib_free(calib_data_t *cal)
{
	switch (cal->type) {
		case typeMagiron:
			matrix_bufFree(&cal->params.magiron.hardCal);
			matrix_bufFree(&cal->params.magiron.softCal);
			break;

		case typeAccorth:
			matrix_bufFree(&cal->params.accorth.offset);
			matrix_bufFree(&cal->params.accorth.ortho);
			break;

		case typeMagmot:
		case typeMotlin:
		default:
			return;
	}
}


int calib_readFile(const char *path, calibType_t type, calib_data_t *cal)
{
	FILE *file;
	int ret;

	if (path == NULL || cal == NULL) {
		return -1;
	}

	file = fopen(path, "r");

	switch (type) {
		case typeMagmot:
			ret = calib_magmotRead(file, cal);
			break;

		case typeMagiron:
			ret = calib_magironRead(file, cal);
			break;

		case typeMotlin:
			ret = calib_motlinRead(file, cal);
			break;

		case typeAccorth:
			ret = calib_accorthRead(file, cal);
			break;

		default:
			fprintf(stderr, "calib: unknown calibration type\n");
			ret = -1;
			break;
	}

	if (file != NULL) {
		fclose(file);
	}

	return ret;
}
