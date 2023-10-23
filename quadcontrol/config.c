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
#include "control.h"
#include <parser.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>


enum { cfg_scenarioID = 0, cfg_pidID, cfg_throttleID, cfg_attitudeID, cfg_attenuateID, cfg_end };

static struct {
	void *data;
	size_t sz;  /* number of elements in `result` */
	int invCnt; /* number of converter invocations */
} res[cfg_end];


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

	/* Due to implementation-defined behavior of realloc when space requested is zero */
	if (res[parserType].invCnt == 0) {
		free(res[parserType].data);
		res[parserType].data = NULL;
		res[parserType].sz = 0;

		/* Returns zero, because not finding a header in particular file is not an error */
		return 0;
	}

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
	err |= parser_fieldGetInt(h, "alt", &mode->takeoff.alt);

	if (err != 0) {
		return -1;
	}

	/* optional parameters */
	if (parser_fieldGetTime(h, "idleT", &mode->takeoff.idleTime) != 0) {
		mode->takeoff.idleTime = 3000;
	}

	if (parser_fieldGetTime(h, "spoolT", &mode->takeoff.spoolTime) != 0) {
		mode->takeoff.idleTime = 3000;
	}

	if (parser_fieldGetTime(h, "liftT", &mode->takeoff.liftTime) != 0) {
		mode->takeoff.idleTime = 2000;
	}

	mode->type = flight_takeoff;

	return 0;
}


static int config_positionParse(const hmap_t *h, flight_mode_t *mode)
{
	int err = 0;

	err |= parser_fieldGetInt(h, "alt", &mode->pos.alt);
	err |= parser_fieldGetInt(h, "lat", &mode->pos.lat);
	err |= parser_fieldGetInt(h, "lon", &mode->pos.lon);

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

	err |= parser_fieldGetInt(h, "alt", &alt);
	err |= parser_fieldGetTime(h, "time", &mode->hover.time);

	if (err != 0 || alt < 0) {
		return -1;
	}

	mode->hover.alt = alt;

	mode->type = flight_hover;

	return 0;
}


static int config_landingParse(const hmap_t *h, flight_mode_t *mode)
{
	if (parser_fieldGetInt(h, "descent", &mode->landing.descent) != 0) {
		mode->landing.descent = 250;
	}

	if (parser_fieldGetInt(h, "diff", &mode->landing.diff) != 0) {
		mode->landing.diff = 4000;
	}

	if (parser_fieldGetTime(h, "timeout", &mode->landing.timeout) != 0) {
		mode->landing.timeout = 5000;
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
	quad_pids_t *pids = (quad_pids_t *)res[cfg_pidID].data;
	pid_ctx_t tmp;
	char *type;

	if (config_reallocData(cfg_pidID, sizeof(quad_pids_t)) != 0) {
		return -1;
	}

	type = hmap_get(h, "type");
	if (type == NULL) {
		fprintf(stderr, "config parser: invalid pid - no `type` in header\n");
		return -1;
	}

	err |= parser_fieldGetFloat(h, "R", &tmp.r.k);
	err |= parser_fieldGetFloat(h, "R_lpf", &tmp.r.f);
	err |= parser_fieldGetFloat(h, "R_max", &tmp.r.max);

	if (tmp.r.max <= 0.0 || tmp.r.f < 0.0 || tmp.r.f >= 1.0) {
		fprintf(stderr, "pid: wrong coeffs %s: %f/%f/%f\n", "R", tmp.r.k, tmp.r.f, tmp.r.max);
		return -1;
	}

	err |= parser_fieldGetFloat(h, "P", &tmp.p.k);
	err |= parser_fieldGetFloat(h, "P_lpf", &tmp.p.f);
	err |= parser_fieldGetFloat(h, "P_max", &tmp.p.max);

	if (tmp.p.max <= 0.0 || tmp.p.f < 0.0 || tmp.p.f >= 1.0) {
		fprintf(stderr, "pid: wrong coeffs %s: %f/%f/%f\n", "P", tmp.p.k, tmp.p.f, tmp.p.max);
		return -1;
	}

	err |= parser_fieldGetFloat(h, "D", &tmp.d.k);
	err |= parser_fieldGetFloat(h, "D_lpf", &tmp.d.f);
	err |= parser_fieldGetFloat(h, "D_max", &tmp.d.max);

	if (tmp.d.max <= 0.0 || tmp.d.f < 0.0 || tmp.d.f >= 1.0) {
		fprintf(stderr, "pid: wrong coeffs %s: %f/%f/%f\n", "D", tmp.d.k, tmp.d.f, tmp.d.max);
		return -1;
	}

	err |= parser_fieldGetFloat(h, "I", &tmp.i.k);
	err |= parser_fieldGetFloat(h, "I_lpf", &tmp.i.f);
	err |= parser_fieldGetFloat(h, "I_max", &tmp.i.max);

	if (tmp.i.max <= 0.0 || tmp.i.f < 0.0 || tmp.i.f >= 1.0) {
		fprintf(stderr, "pid: wrong coeffs %s: %f/%f/%f\n", "I", tmp.i.k, tmp.i.f, tmp.i.max);
		return -1;
	}

	if (err != 0) {
		return -1;
	}

	if (strcmp(type, "roll") == 0) {
		pids->roll = tmp;
	}
	else if (strcmp(type, "pitch") == 0) {
		pids->pitch = tmp;
	}
	else if (strcmp(type, "yaw") == 0) {
		pids->yaw = tmp;
	}
	else if (strcmp(type, "alt") == 0) {
		pids->alt = tmp;
	}
	else if (strcmp(type, "pos") == 0) {
		pids->pos = tmp;
	}
	else {
		fprintf(stderr, "pid: unknown type %s\n", type);
		return -1;
	}

	res[cfg_pidID].invCnt++;

	return 0;
}


int config_pidRead(const char *path, quad_pids_t *ctx, int *sz)
{
	int err;
	parser_t *p;
	static const unsigned int initSz = 5;

	if (path == NULL || ctx == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	*sz = 0;

	/* Parser have to parser one header, which have 13 fields */
	p = parser_alloc(1, 13);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "PID", config_pidConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_pidID].data = malloc(sizeof(quad_pids_t));
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

	if (config_trimUnusedData(cfg_pidID, sizeof(quad_pids_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_pidID].data);
		return -1;
	}

	*sz = res[cfg_pidID].sz;
	*ctx = *(quad_pids_t *)res[cfg_pidID].data;

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

	err |= parser_fieldGetFloat(h, "MAX", &throttle->max);
	err |= parser_fieldGetFloat(h, "MIN", &throttle->min);

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


static int config_attenConverter(const hmap_t *h)
{
	int err = 0;
	mma_atten_t *atten;

	if (config_reallocData(cfg_attenuateID, sizeof(mma_atten_t)) != 0) {
		return -1;
	}

	atten = (mma_atten_t *)res[cfg_attenuateID].data;

	err |= parser_fieldGetFloat(h, "startVal", &atten->startVal);
	err |= parser_fieldGetFloat(h, "endVal", &atten->endVal);
	err |= parser_fieldGetFloat(h, "midVal", &atten->midVal);
	err |= parser_fieldGetFloat(h, "midArg", &atten->midArg);

	if (err != 0) {
		return -1;
	}

	res[cfg_attenuateID].invCnt++;

	return 0;
}


int config_attenRead(const char *path, mma_atten_t **atten, int *sz)
{
	int err;
	parser_t *p;
	static const unsigned int initSz = 1;

	if (path == NULL || atten == NULL || sz == NULL) {
		fprintf(stderr, "config: invalid arguments\n");
		return -1;
	}

	*sz = 0;
	*atten = NULL;

	/* Parser have to parser one header, which have four fields */
	p = parser_alloc(1, 4);
	if (p == NULL) {
		return -1;
	}

	if (parser_headerAdd(p, "ATTENUATE", config_attenConverter) != 0) {
		parser_free(p);
		return -1;
	}

	res[cfg_attenuateID].data = malloc(sizeof(mma_atten_t) * initSz);
	if (res[cfg_attenuateID].data == NULL) {
		parser_free(p);
		return -1;
	}

	res[cfg_attenuateID].invCnt = 0;
	res[cfg_attenuateID].sz = initSz;

	err = parser_execute(p, path, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);
	if (err != 0) {
		free(res[cfg_attenuateID].data);
		return -1;
	}

	if (config_trimUnusedData(cfg_attenuateID, sizeof(mma_atten_t)) != 0) {
		fprintf(stderr, "config: realloc error\n");
		free(res[cfg_attenuateID].data);
		return -1;
	}

	*atten = res[cfg_attenuateID].data;
	*sz = res[cfg_attenuateID].sz;

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

	err |= parser_fieldGetFloat(h, "PITCH", &attitude->pitch);
	err |= parser_fieldGetFloat(h, "ROLL", &attitude->roll);
	err |= parser_fieldGetFloat(h, "YAW", &attitude->yaw);

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
