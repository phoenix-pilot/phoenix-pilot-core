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


enum { cfg_scenarioID = 0, cfg_pidID, cfg_throttleID, cfg_attitudeID, cfg_end };

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


/* Doubles the `res[parserType]` buffer if necessary  */
static int config_reallocData(int parserType, size_t dataTypeSize)
{
	static const int reallocRate = 2;
	void *tmp;

	if (res[parserType].invCnt >= res[parserType].sz) {
		tmp = realloc(res[parserType].data, dataTypeSize * res[parserType].sz * reallocRate);
		if (tmp == NULL) {
			return -1;
		}

		res[parserType].data = tmp;
		res[parserType].sz *= reallocRate;
	}

	return 0;
}


/* If necessary changes the size of res[parserType].data` so that it matches number of elements in this table */
static int config_trimUnusedData(int parserType, size_t dataTypeSize)
{
	void *tmp;

	if (res[parserType].invCnt < res[parserType].sz) {
		tmp = realloc(res[parserType].data, dataTypeSize * res[parserType].invCnt);
		if (tmp == NULL) {
			return -1;
		}

		res[parserType].data = tmp;
		res[parserType].sz = res[parserType].invCnt;
	}

	return 0;
}


static int config_takeoffParse(const hmap_t *h, flight_mode_t *mode)
{
	int err = 0;

	/* obligatory parameters */
	err |= config_parseInt32(h, "alt", &mode->takeoff.alt);

	if (err != 0) {
		return -1;
	}

	/* optional parameters */
	if (config_parseTime(h, "idleT", &mode->takeoff.idleTime) != 0) {
		mode->takeoff.idleTime = 3000;
	}

	if (config_parseTime(h, "spoolT", &mode->takeoff.spoolTime) != 0) {
		mode->takeoff.idleTime = 3000;
	}

	if (config_parseTime(h, "liftT", &mode->takeoff.liftTime) != 0) {
		mode->takeoff.idleTime = 2000;
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
	char *type;
	int err = 0;
	flight_mode_t *mode;

	if (config_reallocData(cfg_scenarioID, sizeof(flight_mode_t)) != 0) {
		return -1;
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

	if (config_trimUnusedData(cfg_scenarioID, sizeof(flight_mode_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_scenarioID].data);
		return -1;
	}

	*scenario = res[cfg_scenarioID].data;
	res[cfg_scenarioID].data = NULL;
	*sz = res[cfg_scenarioID].invCnt;

	return 0;
}


static int config_pidConverter(const hmap_t *h)
{
	int err = 0;
	pid_ctx_t *pid;
	int id = res[cfg_pidID].invCnt;

	if (config_reallocData(cfg_pidID, sizeof(pid_ctx_t)) != 0) {
		return -1;
	}

	pid = (pid_ctx_t *)res[cfg_pidID].data + id;

	err |= config_parseFloat(h, "P", &pid->kp);
	err |= config_parseFloat(h, "I", &pid->ki);
	err |= config_parseFloat(h, "D", &pid->kd);
	err |= config_parseFloat(h, "MIN", &pid->min);
	err |= config_parseFloat(h, "MAX", &pid->max);
	err |= config_parseFloat(h, "IMAX", &pid->maxInteg);
	err |= config_parseFloat(h, "IMIN", &pid->minInteg);

	if (err != 0) {
		return -1;
	}

	res[cfg_pidID].invCnt++;

	return 0;
}


int config_pidRead(const char *path, pid_ctx_t **pids, int *sz)
{
	int err;
	parser_t *p;
	static const unsigned int initSz = 4;

	if (path == NULL || pids == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	*pids = NULL;
	*sz = 0;

	/* Parser have to parser one header, which have seven fields */
	p = parser_alloc(1, 7);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "PID", config_pidConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_pidID].data = malloc(sizeof(pid_ctx_t) * initSz);
	if (res[cfg_pidID].data == NULL) {
		parser_free(p);
		return -1;
	}

	res[cfg_pidID].sz = initSz;
	res[cfg_pidID].invCnt = 0;

	err = parser_execute(p, path, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);
	if (err != 0) {
		free(res[cfg_pidID].data);
		return -1;
	}

	if (config_trimUnusedData(cfg_pidID, sizeof(pid_ctx_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_pidID].data);
		return -1;
	}

	*pids = res[cfg_pidID].data;
	*sz = res[cfg_pidID].sz;

	return 0;
}


static int config_throttleConverter(const hmap_t *h)
{
	int err = 0;
	quad_throttle_t *throttle;
	int id = res[cfg_throttleID].invCnt;

	if (config_reallocData(cfg_throttleID, sizeof(quad_throttle_t)) != 0) {
		return -1;
	}

	throttle = (quad_throttle_t *)res[cfg_throttleID].data + id;

	err |= config_parseFloat(h, "MAX", &throttle->max);
	err |= config_parseFloat(h, "MIN", &throttle->min);

	if (err != 0) {
		return -1;
	}

	res[cfg_throttleID].invCnt++;

	return 0;
}


int config_throttleRead(const char *path, quad_throttle_t **throttle, int *sz)
{
	int err;
	parser_t *p;
	static const unsigned int initSz = 1;

	if (path == NULL || throttle == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	*throttle = NULL;
	*sz = 0;

	/* Parser have to parser one header, which have two fields */
	p = parser_alloc(1, 2);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "THROTTLE", config_throttleConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_throttleID].data = malloc(sizeof(quad_throttle_t) * initSz);
	if (res[cfg_throttleID].data == NULL) {
		parser_free(p);
		return -1;
	}

	res[cfg_throttleID].invCnt = 0;
	res[cfg_throttleID].sz = 1;

	err = parser_execute(p, path, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);
	if (err != 0) {
		free(res[cfg_throttleID].data);
		return -1;
	}

	if (config_trimUnusedData(cfg_throttleID, sizeof(quad_throttle_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_throttleID].data);
		return -1;
	}

	*throttle = res[cfg_throttleID].data;
	*sz = res[cfg_throttleID].sz;

	return 0;
}


static int config_attitudeConverter(const hmap_t *h)
{
	int err = 0;
	quad_att_t *attitude;
	int id = res[cfg_attitudeID].invCnt;

	if (config_reallocData(cfg_attitudeID, sizeof(quad_att_t)) != 0) {
		return -1;
	}

	attitude = (quad_att_t *)res[cfg_attitudeID].data + id;

	err |= config_parseFloat(h, "PITCH", &attitude->pitch);
	err |= config_parseFloat(h, "ROLL", &attitude->roll);
	err |= config_parseFloat(h, "YAW", &attitude->yaw);

	if (err != 0) {
		return -1;
	}

	res[cfg_attitudeID].invCnt++;

	return 0;
}


int config_attitudeRead(const char *path, quad_att_t **attitude, int *sz)
{
	int err;
	parser_t *p;
	static const unsigned int initSz = 1;

	if (path == NULL || attitude == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	*attitude = NULL;
	*sz = 0;

	/* Parser have to parser one header, which have three fields */
	p = parser_alloc(1, 3);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "ATTITUDE", config_attitudeConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_attitudeID].data = malloc(sizeof(quad_att_t) * initSz);
	if (res[cfg_attitudeID].data == NULL) {
		parser_free(p);
		return -1;
	}

	res[cfg_attitudeID].sz = initSz;
	res[cfg_attitudeID].invCnt = 0;

	err = parser_execute(p, path, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);
	if (err != 0) {
		return -1;
	}

	if (config_trimUnusedData(cfg_attitudeID, sizeof(quad_att_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_attitudeID].data);
		return -1;
	}

	*attitude = res[cfg_attitudeID].data;
	*sz = res[cfg_attitudeID].sz;

	return 0;
}
