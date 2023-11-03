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

#ifndef _EKF_FLTR3D_
#define _EKF_FLTR3D_

#include <vec.h>

#define FLTR3D_WDW_LEN 256

#define GYRO_WINDOW_PATH  "etc/ekf_windows/gyro.txt"
#define ACCEL_WINDOW_PATH "etc/ekf_windows/accel.txt"
#define BARO_WINDOW_PATH  "etc/ekf_windows/baro.txt"

typedef struct {
	float window[FLTR3D_WDW_LEN];
	vec_t buf[FLTR3D_WDW_LEN];
	unsigned int windowLen;
	int bufPos;
} fltr3d_ctx_t;


void fltr3d_filter(vec_t *raw, fltr3d_ctx_t *ctx);


int fltr3d_init(const char *path, fltr3d_ctx_t *ctx, const vec_t *initVal);


#endif
