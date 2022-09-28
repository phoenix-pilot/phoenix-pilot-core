/*
 * Phoenix-Pilot
 *
 * Buffers for quaternions library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef QUAT_TEST_BUFF_H
#define QUAT_TEST_BUFF_H

#include <quat.h>

/* Quaternions used for tests */

/* Base quaternions */
static const quat_t QA = { .a = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f };
static const quat_t QI = { .a = 0.0f, .i = 1.0f, .j = 0.0f, .k = 0.0f };
static const quat_t QJ = { .a = 0.0f, .i = 0.0f, .j = 1.0f, .k = 0.0f };
static const quat_t QK = { .a = 0.0f, .i = 0.0f, .j = 0.0f, .k = 1.0f };

static const quat_t Q0 = { .a = 0.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f };

/* Quaternion without zero and one */
static const quat_t Q1 = { .a = 2.0f, .i = 2.0f, .j = 2.0f, .k = 2.0f };

/* Q3 norm must differ from 1 */
static const quat_t Q2 = { .a = 3.0f, .i = 0.0f, .j = 6.0f, .k = 1.0f };
static const quat_t Q3 = { .a = 1.0f, .i = 2.0f, .j = 1.0f, .k = 5.0f };

/* Correct euler angels corresponding to quaternion Q3 (normalized). Hierarchy of angels are yaw, pitch, roll (ZYX) */
static const float Q3roll = 0.5880026;
static const float Q3pitch = -0.6195209;
static const float Q3yaw = 2.5535901;

/* This matrix is equal to Q2 * Q3 */
static const quat_t Q2timesQ3 = { .a = -8.0f, .i = 35.0f, .j = 11.0f, .k = 4.0f };

/* This matrix is equal to Q2 * Q3 * Q2^T */
static const quat_t Q2sandQ3 = { .a = 46.0f, .i = 118.0f, .j = 116.0f, .k = -190.0f };

/* Similar to Q2 and Q3, but with bigger values */
static const quat_t Q4 = { .a = 815.23f, .i = -818.07f, .j = -451.47f, .k = -546.79f };
static const quat_t Q5 = { .a = 334.23f, .i = -822.81f, .j = 349.42f, .k = 548.18f };

/* Correct euler angels corresponding to quaternion Q5 (normalized). Hierarchy of angels are yaw, pitch, roll (ZYX) */
static const float Q5roll = -2.7342767;
static const float Q5pitch = 1.2155062;
static const float Q5yaw = -0.5178249;

/* This matrix is equal to Q2 * Q3 */
static const quat_t Q4timesQ5 = { .a = 56850.1358f, .i = -1000630.3952f, .j = 1032316.741f, .k = -393184.8904f };

/* This matrix is equal to Q2 * Q3 * Q2^T */
static const quat_t Q4sandQ5 = {
	.a = 613861170.772044,
	.i = -27264873.204711974,
	.j = 1092723638.076936,
	.k = -1585712993.277524
};

/* These quaternions are perpendicular do each other (when considered as 4d vectors) */
static const quat_t Q6 = { .a = 0.0f, .i = 1.0f, .j = 5.0f, .k = 2.0f };
static const quat_t Q7 = { .a = 8.0f, .i = 4.0f, .j = 1.0f, .k = -4.5f };

/* Values used for rotations */

/* Zero vector */
static const vec_t V0 = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

/* Base vectors */
static const vec_t VX = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
static const vec_t VY = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
static const vec_t VZ = { .x = 0.0f, .y = 0.0f, .z = 1.0f };

static const vec_t V1 = { .x = 1.0f, .y = 2.0f, .z = 3.0f };
static const vec_t V2 = { .x = -261.48f, .y = 731.11f, .z = -919.51f };

/* Normalized rotation quaternions */
static const quat_t Q8 = { .a = 0.1825742f, .i = 0.3651484f, .j = 0.5477226f, .k = 0.7302967f };

/* Vectors after rotation using Q8 */
static const vec_t V1rotQ8 = { .x = 1.8f, .y = 2.0f, .z = 2.6f };
static const vec_t V2rotQ8 = { .x = -402.5060f, .y = -1031.03f, .z = 472.6079f };

/* Data used in `quat_rotQuat` function tests */
static const vec_t V3 = { .x = 5.0f, .y = 2.0f, .z = 3.0f };

/* Angle in radians */
static const float Angle = 1;

/* Quaternion of rotation about `Angle` and along `V3` axis */
static const quat_t Q9 = { .a = 0.8775826f, .i = 0.3888655f, .j = 0.155546f, .k = 0.2333193f };

/* Vector nearly antiparallel to V10. Antiparallel = parallel, but with opposite directions */
static const vec_t V4 = { .x = -4.52f, .y = -1.44f, .z = -3.0f };

/* Vector nearly parallel to V10 */
static const vec_t V5 = { .x = 7.68f, .y = 2.26f, .z = 5.0f };

/* Data sets for `quat_frameRot` tests */

typedef struct {
	vec_t v1; /* first frame of reference */
	vec_t v2;
	vec_t w1; /* second frame of reference */
	vec_t w2;
	quat_t q1; /* quaternions, which rotates first frame of reference to second one */
	quat_t q2;
	quat_t q1Closer; /* quaternion, which is closer to one quaternion than the other one.  */
	quat_t q2Closer;
} frameRotData_t;

static const frameRotData_t RotExample1 = {
	.v1 = { .x = 0.5883484054145521f, .y = -0.19611613513818404f, .z = 0.7844645405527362f }, /* Normalized (3, -1, 4) vector */
	.v2 = { .x = 0.0f, .y = 0.9701425001453319f, .z = 0.24253562503633297f },                 /* Normalized (0, 4, 1) vector */
	.w1 = { .x = -0.2407717061715384f, .y = 0.1203858530857692f, .z = 0.9630868246861536f },  /* Normalized (-2, 1, 8) vector */
	.w2 = { .x = 0.7043607250604991f, .y = 0.7043607250604991f, .z = 0.08804509063256238f }   /* Normalized (8, 8, 1) vector */
};

static const frameRotData_t RotExample2 = {
	.v1 = { .x = 0.2672612419124244f, .y = 0.5345224838248488f, .z = 0.8017837257372732f },     /* Normalized (1, 2, 3) vector */
	.v2 = { .x = -0.24077170617153842f, .y = 0.8427009716003844f, .z = -0.48154341234307685f }, /* Normalized (-0.8, 2.8, -1.6) vector */
	.w1 = { .x = 0.48107023544236394f, .y = 0.5345224838248488f, .z = 0.6948792289723035f },    /* Normalized (1.8, 2, 2.6) vector */
	.w2 = { .x = 0.24077170617153842f, .y = -0.8427009716003844f, .z = 0.48154341234307685f },  /* Normalized (0.8, -2.8, 1.6) vector */
	.q1 = { .a = 0.0f, .i = 0.3768674f, .j = 0.538382f, .k = 0.753735f },
	.q2 = { .a = -0.0f, .i = -0.3768674f, .j = -0.538382f, .k = -0.753735f },
	.q1Closer = { .a = 0.0f, .i = 0.0f, .j = 1.0f, .k = 0.0f },
	.q2Closer = { .a = 0.0f, .i = 0.0f, .j = -1.0f, .k = 0.0f }
};

#endif
