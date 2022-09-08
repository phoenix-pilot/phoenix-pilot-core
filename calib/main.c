/*
 * Phoenix-Pilot
 *
 * main.c: Drone calibration module
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
#include <stdbool.h>
#include <errno.h>

#include <calibcore.h>
#include <hmap.h>
#include <calib.h>


/* text formatting defined */
#define COLOR_BOLD "\e[1m"
#define COLOR_OFF  "\e[m"


static void calib_help(hmap_t *calibs)
{
	unsigned int iter;
	calib_t *cal;

	printf("Usage: calibtool mode [ARGS]\n");
	printf(
		"  %i calibration mode%savailable%c\n\n",
		calibs->size,
		(calibs->size == 1) ? " " : "s ",
		(calibs->size == 0) ? '.' : ':');

	/* iterating over hashmap */
	iter = 0;
	cal = (calib_t *)hmap_next(calibs, &iter);
	while (cal != NULL) {
		if (cal->help == NULL) {
			fprintf(stderr, "calibtool: calibration %s lacks help function\n", cal->name);
		}
		else {
			printf("  %s%s%s:\n  %s\n", COLOR_BOLD, cal->name, COLOR_OFF, cal->help());
		}

		cal = (calib_t *)hmap_next(calibs, &iter);
	}
}


/* 
* Overwrites the calibration file with values from known calibration procedure modules. 
* Prints @tag itself and asks each module to print its values 
*/
static int calib_write(const char *path, hmap_t *calibs)
{
	int ret = EOK;
	unsigned int iter;
	FILE *file;
	calib_t *cal;

	file = fopen(path, "w");
	if (file == NULL) {
		file = stdout;
		fprintf(stderr, "calibtool: error during %s opening. Printing calib to console\n", path);
		ret = -ENFILE;
	}


	/* iterating over hashmap */
	iter = 0;
	cal = (calib_t *)hmap_next(calibs, &iter);
	while (cal != NULL) {
		fprintf(file, "@%s\n", cal->name); /* print tag */
		cal->write(file);                  /* call for data print */
		fprintf(file, "\n\n");             /* print newlines for separation */

		cal = (calib_t *)hmap_next(calibs, &iter);
	}

	if (file != stdout) {
		fclose(file);
	}

	return ret;
}


/* calibration routine scheme */
static int calib_do(calib_t *cal, int argc, const char **argv)
{
	int ret;

	if (cal == NULL) {
		return -ENOENT;
	}

	if (cal->proc.calib.init == NULL || cal->proc.calib.run == NULL || cal->proc.calib.done == NULL) {
		return -EINVAL;
	}

	ret = cal->proc.calib.init(argc, argv);
	if (ret != EOK) {
		fprintf(stderr, "calibtool: procedure '%s' init failed with code: %d\n", cal->name, ret);
		return ret;
	}
	ret = cal->proc.calib.run();
	cal->proc.calib.done();

	if (ret != EOK) {
		fprintf(stderr, "calibtool: procedure '%s' failed with code: %d\n", cal->name, ret);
		return ret;
	}

	fprintf(stdout, "calibtool: calibration finished successfully!\n");
	return EOK;
}


int main(int argc, const char **argv)
{
	calib_t *cal;
	hmap_t *calibs;

	calibs = calib_hashmapGet();

	if (argc <= 1) {
		fprintf(stderr, "calibtool: wrong arguments.\n");
		calib_help(calibs);
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "-h") == 0) {
		calib_help(calibs);
		return EXIT_SUCCESS;
	}

	/* get calibration procedure according to user choice */
	cal = hmap_get(calibs, argv[1]);
	if (cal == NULL) {
		fprintf(stderr, "calibtool: unknown procedure '%s'. Use option '-h' to print help.\n", argv[1]);
		return EXIT_FAILURE;
	}

	/* Read calibration file */
	calib_read(CALIB_FILE, calibs);

	/* Perform calibration */
	if (calib_do(cal, argc, argv) != EOK) {
		return EXIT_FAILURE;
	}

	/* Write updated calibration parameters to file */
	if (calib_write(CALIB_FILE, calibs) != EOK) {
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
