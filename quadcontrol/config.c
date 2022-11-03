/*
 * Phoenix-Pilot
 *
 * Config parsers
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "config.h"
#include <parser.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>


enum { cfg_scenarioID = 0, cfg_pidID, cfg_end };


static struct {
	void *data;
	size_t sz;  /* number of elements in `result` */
	int invCnt; /* number of converter invocations */
} res[cfg_end];


static int config_takeoffParse(const hmap_t *h, flight_mode_t *mode)
{
	char *altStr, *timeStr, *endptr;

	altStr = hmap_get(h, "alt");
	timeStr = hmap_get(h, "time");

	if (altStr == NULL || timeStr == NULL) {
		fprintf(stderr, "config: not all required fields in takeoff header\n");
		return -1;
	}

	mode->takeoff.alt = strtol(altStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid altitude value in takeoff header\n");
		return -1;
	}

	mode->takeoff.time = strtof(timeStr, &endptr);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid time value in takeoff header\n");
		return -1;
	}

	mode->type = flight_takeoff;

	return 0;
}


static int config_positionParse(const hmap_t *h, flight_mode_t *mode)
{
	char *valStr, *endptr;

	valStr = hmap_get(h, "alt");
	if (valStr == NULL) {
		fprintf(stderr, "config: no \"alt\" field in position header\n");
		return -1;
	}

	mode->pos.alt = strtol(valStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid altitude value in position header\n");
		return -1;
	}

	valStr = hmap_get(h, "lat");
	if (valStr == NULL) {
		fprintf(stderr, "config: no \"lat\" field in position header\n");
		return -1;
	}

	mode->pos.lat = strtol(valStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid latitude value in position header\n");
		return -1;
	}

	valStr = hmap_get(h, "lon");
	if (valStr == NULL) {
		fprintf(stderr, "config: no \"lon\" field in position header\n");
		return -1;
	}

	mode->pos.lon = strtol(valStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid longitude value in position header\n");
		return -1;
	}

	mode->type = flight_pos;

	return 0;
}


static int config_hoverParse(const hmap_t *h, flight_mode_t *mode)
{
	char *altStr, *timeStr, *endptr;

	altStr = hmap_get(h, "alt");
	timeStr = hmap_get(h, "time");

	if (altStr == NULL || timeStr == NULL) {
		fprintf(stderr, "config: not all required fields in hover header\n");
		return -1;
	}

	mode->hover.alt = strtol(altStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid altitude value in hover header\n");
		return -1;
	}

	mode->hover.time = strtoull(timeStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid time value in hover header\n");
		return -1;
	}

	mode->type = flight_hover;

	return 0;
}


static int config_landingParse(const hmap_t *h, flight_mode_t *mode)
{
	char *timeStr, *endptr;

	timeStr = hmap_get(h, "time");

	if (timeStr == NULL) {
		fprintf(stderr, "config: no \"time\" field in landing header\n");
		return -1;
	}

	mode->landing.time = strtoull(timeStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid \"time\" value in hover header\n");
		return -1;
	}

	mode->type = flight_landing;

	return 0;
}


static int config_scenarioConverter(const hmap_t *h)
{
	static const int reallocRate = 2;
	char *type;
	void *tmp;
	int err = 0;
	flight_mode_t *mode;

	if (res[cfg_scenarioID].invCnt >= res[cfg_scenarioID].sz) {
		/* Doubling space if current is too small */
		tmp = realloc(res[cfg_scenarioID].data, sizeof(flight_mode_t) * res[cfg_scenarioID].sz * reallocRate);
		if (tmp == NULL) {
			return -1;
		}

		res[cfg_scenarioID].data = tmp;
		res[cfg_scenarioID].sz *= reallocRate;
	}

	mode = (flight_mode_t *)res[cfg_scenarioID].data + res[cfg_scenarioID].invCnt;

	type = hmap_get(h, "type");
	if (type == NULL) {
		fprintf(stderr, "config parser: invalid file - no `type` in header\n");
		return -1;
	}


	if (strcmp(type, "flight_takeoff") == 0) {
		err = config_takeoffParse(h, mode);
	}
	else if (strcmp(type, "flight_position") == 0) {
		err = config_positionParse(h, mode);
	}
	else if (strcmp(type, "flight_hover") == 0) {
		err = config_hoverParse(h, mode);
	}
	else if (strcmp(type, "flight_landing") == 0) {
		err = config_landingParse(h, mode);
	}
	else if (strcmp(type, "flight_end") == 0) {
		mode->type = flight_end;
	}
	else if (strcmp(type, "flight_manual") == 0) {
		mode->type = flight_manual;
	}
	else if (strcmp(type, "flight_manualAbort") == 0) {
		mode->type = flight_manualAbort;
	}
	else {
		fprintf(stderr, "config: not recognised type\n");
		return -1;
	}

	if (err != 0) {
		return -1;
	}

	res[cfg_scenarioID].invCnt++;

	return 0;
}


int config_scenarioRead(const char *path, flight_mode_t **scenario, size_t *sz)
{
	void *tmp;
	int err = 0;
	static const unsigned int initSz = 5;

	if (path == NULL || scenario == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	parser_t *p = parser_alloc(1, 3);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "flight_mode", config_scenarioConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_scenarioID].data = malloc(sizeof(flight_mode_t) * initSz);
	if (res[cfg_scenarioID].data == NULL) {
		parser_free(p);
		return -1;
	}

	res[cfg_scenarioID].sz = initSz;
	res[cfg_scenarioID].invCnt = 0;

	err = parser_execute(p, path);
	parser_free(p);
	if (err != 0) {
		free(res[cfg_scenarioID].data);
		return -1;
	}

	if (res[cfg_scenarioID].invCnt < res[cfg_scenarioID].sz) {
		tmp = realloc(res[cfg_scenarioID].data, sizeof(flight_mode_t) * res[cfg_scenarioID].invCnt);
		if (tmp == NULL) {
			fprintf(stderr, "config: realloc error\n");
			free(res[cfg_scenarioID].data);
			return -1;
		}

		res[cfg_scenarioID].data = tmp;
	}

	*scenario = res[cfg_scenarioID].data;
	res[cfg_scenarioID].data = NULL;
	*sz = res[cfg_scenarioID].invCnt;

	return 0;
}
