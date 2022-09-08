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

#include "calib.h"
#include "hmap.h"

#define CALIB_FILE  "/etc/calib.conf" /* Path to calibration parameters file */

/* text formatting defined */
#define COLOR_BOLD "\e[1m"
#define COLOR_OFF  "\e[m"


struct {
	hmap *calibs;
} calib_common;


/* Get calibration procedure named `key` */
static inline calib_t *calib_get(const char *key)
{
	return hmap_get(calib_common.calibs, key);
}


/* Register new calib_t procedure. */
void calib_register(calib_t *c)
{
	if (hmap_insert(calib_common.calibs, c->name, c) < 0) {
		fprintf(stderr, "calibtool: ailed to register %s procedure\n", c->name);
	}
}


static void calib_help(void)
{
	unsigned int iter;
	calib_t *cal;

	printf("Usage: calibtool mode [ARGS]\n");
	printf(
		"  %i calibration mode%savailable%c\n\n",
		calib_common.calibs->used,
		(calib_common.calibs->used == 1) ? " " : "s ",
		(calib_common.calibs->used == 0) ? '.' : ':');

	/* iterating over hashmap */
	iter = 0;
	cal = (calib_t *)hmap_next(calib_common.calibs, &iter);
	while (cal != NULL) {
		printf("  %s%s%s:\n%s\n", COLOR_BOLD, cal->name, COLOR_OFF, cal->help());
		cal = (calib_t *)hmap_next(calib_common.calibs, &iter);
	}
}


/* 
* Reads calibration file, and given a @tag sends preformatted pair param/value 
* to correct calibration procedure interpreter 
*/
static int calib_read(const char *path)
{
	FILE *file;
	calib_t *cal;
	char *head, *val, *line = NULL;
	size_t lineSz = 0;
	unsigned int lineNum = 0;
	int ret = EOK;

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
			cal = calib_get(&head[1]);
			if (cal == NULL) {
				fprintf(stderr, "calibtool: error reading %s at line %i: unknown calib mode %s \n", path, lineNum, head);
				ret = -ENOENT;
				break;
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


/* 
* Overwrites the calibration file with values from known calibration procedure modules. 
* Prints @tag itself and asks each module to print its values 
*/
static int calib_write(const char *path)
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
	cal = (calib_t *)hmap_next(calib_common.calibs, &iter);
	while (cal != NULL) {
		fprintf(file, "@%s\n", cal->name); /* print tag */
		cal->write(file);                  /* call for data print */
		fprintf(file, "\n\n");             /* print newlines for separation */

		cal = (calib_t *)hmap_next(calib_common.calibs, &iter);
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
	calib_t *cal;

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
	cal = calib_get(argv[1]);
	if (cal == NULL) {
		fprintf(stderr, "calibtool: unknown procedure '%s'. Use option '-h' to print help.\n", argv[1]);
		return EXIT_FAILURE;
	}

	/* Read calibration file */
	calib_read(CALIB_FILE);

	/* Perform calibration */
	if (calib_do(cal, argc, argv) != EOK) {
		return EXIT_FAILURE;
	}

	/* Write updated calibration parameters to file */
	if (calib_write(CALIB_FILE) != EOK) {
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
