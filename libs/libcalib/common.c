/*
 * Phoenix-Pilot
 *
 * main.c: Drone calibration library
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
#include <errno.h>

#include "calib.h"
#include "hmap.h"

/* 
* Reads calibration file, and given a @tag sends preformatted pair param/value 
* to correct calibration procedure interpreter. 
*
* Uses calibration procedures from `calibs` hashmap
*/
int calib_read(const char *path, hmap_t *calibs)
{
	FILE *file;
	calib_t *cal;
	char *head, *val, *line = NULL;
	size_t lineSz = 0;
	unsigned int lineNum = 0;
	int ret = EOK;

	if (calibs == NULL) {
		return -EINVAL;
	}

	file = fopen(path, "r+");
	if (file == NULL) {
		/* it is ok, maybe the file is just missing */
		fprintf(stderr, "calibtool: %s not found. Continuing...\n", path);
		return EOK;
	}

	cal = NULL;
	while (getline(&line, &lineSz, file) >= 0) {
		head = strtok(line, " \n");

		/* strange error */
		if (head == NULL) {
			fprintf(stderr, "calibtool: error reading %s at line %i:\n", path, lineNum);
			ret = -ENOENT;
			break;
		}

		if (head[0] == '\n' || head[0] == '#') {
			/* line skip conditions */
			continue;
		}
		else if (head[0] == '@') {
			/* tag condition */
			cal = hmap_get(calibs, &head[1]);
			if (cal == NULL) {
				fprintf(stderr, "calibtool: error reading %s at line %i: unknown calib mode %s \n", path, lineNum, head);
				ret = -ENOENT;
				break;
			}
			if (cal->interpret == NULL) {
				fprintf(stderr, "calibtool: calibration %s lacks interpreter\n", cal->name);
			}
		}
		else {
			/* normal line condition */
			val = strtok(NULL, " \n");

			if (val == NULL || cal == NULL) {
				fprintf(
					stderr,
					"calibtool: error reading %s at line %i:\nno calibmode tag found yet, or lack of value for %s\n",
					path, lineNum, head);
				ret = -ENOENT;
				break;
			}

			if (cal->interpret(head, atof(val)) != EOK) {
				fprintf(
					stderr,
					"calibtool: error reading %s at line %i:\ncalibmode %s can't interpret name/value pair:%s/%s\n",
					path, lineNum, cal->name, head, val);
				ret = -EINVAL;
				break;
			}
		}

		lineNum++;
	}

	free(line);
	fclose(file);

	return ret;
}