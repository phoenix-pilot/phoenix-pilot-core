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

#include <hmap.h>

#include "calibtool.h"

#define PATH_CALIB_FILE "/etc/calib.conf" /* Path to calibration parameters file */
#define CALIBS_SIZE     16                /* Maximum number of calibrations available. Can be freely increased */

/* text formatting defined */
#define COLOR_BOLD "\e[1m"
#define COLOR_OFF  "\e[m"


struct {
	hmap_t *calibs;
} calib_common;


/* Register new calib_ops_t procedure. */
void calib_register(calib_ops_t *c)
{
	if (hmap_insert(calib_common.calibs, c->name, c) < 0) {
		fprintf(stderr, "calibtool: ailed to register %s procedure\n", c->name);
	}
}


static void calib_help(void)
{
	unsigned int iter;
	calib_ops_t *cal;

	printf("Usage: calibtool mode [ARGS]\n");
	printf(
		"  %i calibration mode%savailable%c\n\n",
		calib_common.calibs->size,
		(calib_common.calibs->size == 1) ? " " : "s ",
		(calib_common.calibs->size == 0) ? '.' : ':');

	/* iterating over hashmap */
	iter = 0;
	cal = (calib_ops_t *)hmap_next(calib_common.calibs, &iter);
	while (cal != NULL) {
		if (cal->help == NULL) {
			fprintf(stderr, "calibtool: calibration %s lacks help function\n", cal->name);
		}
		else {
			printf("  %s%s%s: %s", COLOR_BOLD, cal->name, COLOR_OFF, cal->help());
		}

		cal = (calib_ops_t *)hmap_next(calib_common.calibs, &iter);
	}
}


/* 
* Reads calibration file, and given a @tag sends preformatted pair param/value 
* to correct calibration procedure interpreter 
*/
static int calib_read(const char *path)
{
	calib_ops_t *c;
	calib_data_t *calib;
	unsigned int i = 0, inited = 0;
	bool err = false;

	c = (calib_ops_t *)hmap_next(calib_common.calibs, &i);
	while (c != NULL) {
		if (c->dataGet != NULL) {
			calib = c->dataGet();
			if (calib_dataInit(path, calib->type, calib) != 0) {
				err = true;
				break;
			}
		}
		inited++;

		c = (calib_ops_t *)hmap_next(calib_common.calibs, &i);
	}

	/* deinitialize all calibrations up to the failed one */
	if (err) {
		i = 0;
		while (inited > 0) {
			c = (calib_ops_t *)hmap_next(calib_common.calibs, &i);
			if (c->dataGet != NULL) {
				calib = c->dataGet();
				calib_free(calib);
			}
			inited--;
		}
		return -1;
	}

	return 0;
}


/* 
* Overwrites the calibration file with values from known calibration procedure modules. 
* Prints @tag itself and asks each module to print its values 
*/
static int calib_write(const char *path)
{
	int ret = EOK;
	unsigned int iter;
	FILE *file;
	calib_ops_t *cal;

	file = fopen(path, "w");
	if (file == NULL) {
		file = stdout;
		fprintf(stderr, "calibtool: error during %s opening. Printing calib to console\n", path);
		ret = -ENFILE;
	}


	/* iterating over hashmap */
	iter = 0;
	cal = (calib_ops_t *)hmap_next(calib_common.calibs, &iter);
	while (cal != NULL) {
		if (cal->write != NULL) {
			fprintf(file, "@%s\n", cal->name); /* print tag */
			cal->write(file);                  /* call for data print */
			fprintf(file, "\n\n");             /* print newlines for separation */
		}
		cal = (calib_ops_t *)hmap_next(calib_common.calibs, &iter);
	}

	if (file != stdout) {
		fclose(file);
	}

	return ret;
}


/* calibration routine scheme */
static int calib_run(calib_ops_t *cal, int argc, const char **argv)
{
	int ret;

	if (cal == NULL) {
		return -ENOENT;
	}

	if (cal->init == NULL || cal->run == NULL || cal->done == NULL) {
		return -EINVAL;
	}

	ret = cal->init(argc, argv);
	if (ret != EOK) {
		fprintf(stderr, "calibtool: procedure '%s' init failed with code: %d\n", cal->name, ret);
		return ret;
	}
	ret = cal->run();
	cal->done();

	if (ret != EOK) {
		fprintf(stderr, "calibtool: procedure '%s' failed with code: %d\n", cal->name, ret);
		return ret;
	}

	fprintf(stdout, "calibtool: calibration finished successfully!\n");
	return EOK;
}


int main(int argc, const char **argv)
{
	calib_ops_t *cal;

	if (argc <= 1) {
		fprintf(stderr, "calibtool: wrong arguments.\n");
		calib_help();
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "-h") == 0) {
		calib_help();
		return EXIT_SUCCESS;
	}

	/* get calibration procedure according to user choice */
	cal = hmap_get(calib_common.calibs, argv[1]);
	if (cal == NULL) {
		fprintf(stderr, "calibtool: unknown procedure '%s'. Use option '-h' to print help.\n", argv[1]);
		return EXIT_FAILURE;
	}

	/* Read calibration file */
	if (calib_read(PATH_CALIB_FILE) != 0) {
		fprintf(stderr, "calibtool: error on calibrations initialization\n");
		return EXIT_FAILURE;
	}

	/* Perform calibration */
	if (calib_run(cal, argc, argv) != EOK) {
		return EXIT_FAILURE;
	}

	/* Write updated calibration parameters to file */
	if (calib_write(PATH_CALIB_FILE) != EOK) {
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


/* This constructor must run before calibration procedures constructors. It assures NULL-ability of hashmap */
__attribute__((constructor(101))) static void calib_premain(void)
{
	calib_common.calibs = hmap_init(CALIBS_SIZE);

	if (calib_common.calibs == NULL) {
		printf("calibtool: hashmap allocation fail!");
		exit(EXIT_FAILURE);
	}
}


__attribute__((destructor(101))) static void calib_postmain(void)
{
	hmap_free(calib_common.calibs);
}
