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
			if (row > (unsigned int)'9' || col > (unsigned int)'9') {
				return -1;
			}
			mat = &cal->params.magiron.softCal;
			break;

		case CHAR_HARDIRON:
			if (row > (unsigned int)'3' || col > (unsigned int)'3') {
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
	int motor, param;

	if (strlen(paramName) != 4) {
		return -1;
	}

	motor = (int8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */
	param = (int8_t)(paramName[3] - 'a'); /* convert character to unsigned int digit, expected range [a, b] -> [0, 1] */


	if (motor >= NUM_OF_MOTORS || motor < 0) {
		return -1;
	}
	if (param < 0 || param > 1) {
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


static int calib_accrotEnter(const char *paramName, calib_data_t *cal, float val)
{
	unsigned int qParam;

	if (strlen(paramName) != 5) {
		return -1;
	}

	if (strncmp(paramName, "accq", strlen("accq")) != 0) {
		return -1;
	}

	qParam = (uint8_t)(paramName[4] - '0'); /* convert character to unsigned int digit */

	/* quaternion variable select */
	switch (qParam) {
		case 0:
			cal->params.accrot.frameQ.a = val;
			break;
		case 1:
			cal->params.accrot.frameQ.i = val;
			break;
		case 2:
			cal->params.accrot.frameQ.j = val;
			break;
		case 3:
			cal->params.accrot.frameQ.k = val;
			break;
		default:
			/* only support for "accq0", "accq1", "accq2", "accq3" */
			fprintf(stderr, "%s: wrong quaternion variable\n", ACCROT_TAG);
			return -1;
	}

	return 0;
}


static inline void calib_accrotDefaults(calib_data_t *cal)
{
	/* Identity quaternion as default (no ratation) */
	quat_idenWrite(&cal->params.accrot.frameQ);
}


static int calib_accrotRead(FILE *file, calib_data_t *cal)
{
	char *line, *name;
	size_t lineSz;
	float val = 0, diff;
	unsigned int params = 0; /* can be easily mislead correct number of wrong parameters, but better than nothing */

	calib_accrotDefaults(cal);

	if (file == NULL) {
		fprintf(stderr, "No calibration file. '%s' going default.\n", ACCROT_TAG);
		return 0;
	}

	/* Scroll to 'magiron' tag */
	if (calib_file2tag(file, ACCROT_TAG) != 0) {
		fprintf(stderr, "Calibration not done yet. '%s' going default.\n", ACCROT_TAG);
		return 0;
	}

	line = NULL;
	while (calib_getline(&line, &lineSz, file, &name, &val) == 0) {
		if (calib_accrotEnter(name, cal, val) != 0) {
			break;
		}
		params++;
	}
	free(line);

	if (params != ACCROT_PARAMS) {
		calib_accrotDefaults(cal);
		fprintf(stderr, "Failed to read `%s` calibration. Going default.\n", ACCROT_TAG);
	}

	diff = 1.f - quat_len(&cal->params.accrot.frameQ);
	if (diff > ACCROT_QUAT_ERR || diff < -ACCROT_QUAT_ERR) {
		fprintf(stderr, "calib %s: wrong quaternion norm of %f\n", ACCROT_TAG, 1.f - diff);
		return -1;
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

		case typeMagmot:
		case typeMotlin:
		case typeAccrot:
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

		case typeAccrot:
			ret = calib_accrotRead(file, cal);
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
