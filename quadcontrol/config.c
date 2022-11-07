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


static int config_parseInt32(const hmap_t *h, char *fieldName, int32_t *target)
{
	char *valueStr, *endptr;

	valueStr = hmap_get(h, fieldName);
	if (valueStr == NULL) {
		fprintf(stderr, "config: no \"%s\" field in header\n", fieldName);
		return -1;
	}

	*target = strtol(valueStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid \"%s\" value in header\n", fieldName);
		return -1;
	}

	return 0;
}


static int config_parseFloat(const hmap_t *h, char *fieldName, float *target)
{
	char *valueStr, *endptr;

	valueStr = hmap_get(h, fieldName);
	if (valueStr == NULL) {
		fprintf(stderr, "config: no \"%s\" field in header\n", fieldName);
		return -1;
	}

	*target = strtof(valueStr, &endptr);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid \"%s\" value in header\n", fieldName);
		return -1;
	}

	return 0;
}

static int config_parseTime(const hmap_t *h, char *fieldName, time_t *target)
{
	char *valueStr, *endptr;

	valueStr = hmap_get(h, fieldName);
	if (valueStr == NULL) {
		fprintf(stderr, "config: no \"%s\" field in header\n", fieldName);
		return -1;
	}

	*target = strtoull(valueStr, &endptr, 10);
	if (endptr[0] != '\0') {
		fprintf(stderr, "config: invalid \"%s\" value in header\n", fieldName);
		return -1;
	}

	return 0;
}


static int config_takeoffParse(const hmap_t *h, flight_mode_t *mode)
{
	int err = 0;

	err |= config_parseInt32(h, "alt", &mode->takeoff.alt);
	err |= config_parseFloat(h, "time", &mode->takeoff.time);

	if (err != 0) {
		return -1;
	}

	mode->type = flight_takeoff;

	return 0;
}


static int config_positionParse(const hmap_t *h, flight_mode_t *mode)
{
	int err = 0;

	err |= config_parseInt32(h, "alt", &mode->pos.alt);
	err |= config_parseInt32(h, "lat", &mode->pos.lat);
	err |= config_parseInt32(h, "lon", &mode->pos.lon);

	if (err != 0) {
		return -1;
	}

	mode->type = flight_pos;

	return 0;
}


static int config_hoverParse(const hmap_t *h, flight_mode_t *mode)
{
	int err = 0;
	int32_t alt;

	err |= config_parseInt32(h, "alt", &alt);
	err |= config_parseTime(h, "time", &mode->hover.time);

	if (err != 0 || alt < 0) {
		return -1;
	}

	mode->hover.alt = alt;

	mode->type = flight_hover;

	return 0;
}


static int config_landingParse(const hmap_t *h, flight_mode_t *mode)
{
	if (config_parseTime(h, "time", &mode->landing.time) != 0) {
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

	*scenario = NULL;
	*sz = 0;

	/* Parser have to parse one header with two fields */
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

	err = parser_execute(p, path, PARSER_EXEC_ALL_HEADERS);
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
