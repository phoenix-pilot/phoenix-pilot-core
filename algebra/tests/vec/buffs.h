/*
 * Phoenix-Pilot
 *
 * Buffers for vectors library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef VEC_TEST_BUFF_H
#define VEC_TEST_BUFF_H

#include <vec.h>

/* Values used for vectors library tests */

/* Zero vector */
static const vec_t V0 = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

/* Base vectors */
static const vec_t VX = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
static const vec_t VY = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
static const vec_t VZ = { .x = 0.0f, .y = 0.0f, .z = 1.0f };

/* Small values */
static const vec_t V1 = { .x = 1.0f, .y = 2.0f, .z = 3.0f };
static const vec_t V2 = { .x = 4.0f, .y = 5.0f, .z = 6.0f };

/* More complicated values. Length of this vectors must be bigger than 1 */
static const vec_t V3 = { .x = -261.48f, .y = 731.11f, .z = -919.51f };
static const vec_t V4 = { .x = 613.36f, .y = -708.58f, .z = -150.27f };

/* This vectors are perpendicular to each other */
static const vec_t V5 = { .x = 0.0f, .y = 1.0f, .z = 4.0f };
static const vec_t V6 = { .x = 5.0f, .y = -8.0f, .z = 2.0f };

/* Length of this vector must be smaller than 1 */
static const vec_t V7 = { .x = 0.25f, .y = 0.5f, .z = 0.5f };

#endif
