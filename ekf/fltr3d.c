/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * generic vector filter
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <vec.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>

#include "fltr3d.h"


#define LINE_LEN 32


void fltr3d_filter(vec_t *raw, fltr3d_ctx_t *ctx)
{
	vec_t part, full = { 0 };
	int i, j;

	if (raw == NULL) {
		for (i = 0; i < ctx->windowLen; i++) {
			ctx->buf[i].x = ctx->buf[i].y = ctx->buf[i].z = 0;
		}
		return;
	}

	ctx->buf[ctx->bufPos] = *raw;

	for (i = 0; i < ctx->windowLen; i++) {
		j = (ctx->bufPos) - i;
		if (j < 0) {
			j += ctx->windowLen;
		}

		part = (i > ctx->bufPos) ? ctx->buf[ctx->windowLen + ctx->bufPos - i] : ctx->buf[(ctx->bufPos) - i];
		vec_times(&part, ctx->window[ctx->windowLen - 1 - i]);
		vec_add(&full, &part);
	}

	/* Cyclic increment */
	ctx->bufPos += 1;
	if ((ctx->bufPos) == ctx->windowLen) {
		ctx->bufPos = 0;
	}

	*raw = full;
}


static void fltr3d_initBuffer(fltr3d_ctx_t *ctx, const vec_t *initVec)
{
	vec_t v = { 0 };
	int i;

	if (initVec != NULL) {
		v = *initVec;
	}

	for (i = 0; i < ctx->windowLen; i++) {
		ctx->buf[i] = v;
	}

	ctx->bufPos = 0;
}


static int fltr3d_checkWindow(fltr3d_ctx_t *ctx)
{
	float sum = 0;
	int i;

	for (i = 0; i < ctx->windowLen; i++) {
		sum += ctx->window[i];
	}

	/* Filtering window sum must be 1 to not change the amplitude of signal */
	if (sum > 1.01 || sum < 0.99) {
		fprintf(stderr, "fltr3d: ubalanced window: sum = %f\n", sum);
		return -1;
	}

	return 0;
}


static int fltr3d_readWindow(FILE *fp, fltr3d_ctx_t *ctx)
{
	char line[LINE_LEN] = { 0 };
	int c = 0, len, i;
	float val = 0;

	c = 0;
	i = 0;
	while ((c != EOF) && (i < FLTR3D_WDW_LEN)) {
		len = 0;

		while (len < LINE_LEN - 1) {
			c = fgetc(fp);
			if (isdigit(c) == 0 && c != '.' && c != '+' && c != '-') {
				break;
			}

			line[len] = c;
			len++;
		}

		line[len] = '\0';

		if (sscanf(line, "%f", &val) != 1) {
			fprintf(stderr, "fltr3d: error in line %d\n", i);
			return -1;
		}
		ctx->window[i] = val;
		ctx->windowLen = ++i;
	}

	return 0;
}


int fltr3d_init(const char *path, fltr3d_ctx_t *ctx, const vec_t *initVal)
{
	FILE *fp;

	if (ctx == NULL) {
		fprintf(stderr, "fltr3d: null detected\n");
		return -1;
	}

	fp = fopen(path, "r");
	if (fp == NULL) {
		fprintf(stderr, "fltr3d: can`t open %s\n", path);
		return -1;
	}

	if (fltr3d_readWindow(fp, ctx) != 0) {
		fprintf(stderr, "fltr3d: can`t read %s\n", path);
		return -1;
	}
	fclose(fp);

	if (fltr3d_checkWindow(ctx) != 0) {
		fprintf(stderr, "fltr3d: invalid file %s\n", path);
		return -1;
	}

	fltr3d_initBuffer(ctx, initVal);

	return 0;
}
