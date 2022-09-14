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


static int calib_magmotRead(FILE *file, calib_t *cal)
{
	if (calib_file2tag(file, "magmot") != 0) {
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
