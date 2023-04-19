/*
 * Phoenix-Pilot
 *
 * Buffers for quaternions differentiation library unit tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef QDIFF_TEST_BUFF_H
#define QDIFF_TEST_BUFF_H

/* Init value for testing matrix */
static const float matInitBuff[] = { -1.0f };

/* Quaternions used for tests */

/* Base quaternions */
static const quat_t QA = { .a = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f };
static const quat_t QI = { .a = 0.0f, .i = 1.0f, .j = 0.0f, .k = 0.0f };
static const quat_t QJ = { .a = 0.0f, .i = 0.0f, .j = 1.0f, .k = 0.0f };
static const quat_t QK = { .a = 0.0f, .i = 0.0f, .j = 0.0f, .k = 1.0f };

static const quat_t A = { .a = -9.0f, .i = -8.0f, .j = 7.0f, .k = 1.0f };
static const quat_t B = { .a = 69.3764f, .i = 18.6179f, .j = 78.7688f, .k = -90.3680f };

/* clang-format off */

/* Result of differentiation QI*p with respect to QI, where p is a quaternion */
static const float buffs_QIpDiffQI[] = {
    0.0f, -1.0f,  0.0f, 0.0f,
    1.0f,  0.0f,  0.0f, 0.0f,
    0.0f,  0.0f,  0.0f, 1.0f,
    0.0f,  0.0f, -1.0f, 0.0f
};

/* Result of differentiation QJ*p with respect to QJ, where p is a quaternion */
static const float buffs_QJpDiffQJ[] = {
    0.0f, 0.0f, -1.0f,  0.0f,
    0.0f, 0.0f,  0.0f, -1.0f,
    1.0f, 0.0f,  0.0f,  0.0f,
    0.0f, 1.0f,  0.0f,  0.0f
};

/* Result of differentiation QK*p with respect to QK, where p is a quaternion */
static const float buffs_QKpDiffQK[] = {
    0.0f,  0.0f, 0.0f, -1.0f,
    0.0f,  0.0f, 1.0f,  0.0f,
    0.0f, -1.0f, 0.0f,  0.0f,
    1.0f,  0.0f, 0.0f,  0.0f
};

/* Result of differentiation A*p with respect to A, where p is a quaternion */
static const float buffs_ApDiffA[] = {
    -9.0f,  8.0f, -7.0f, -1.0f,
    -8.0f, -9.0f,  1.0f, -7.0f,
     7.0f, -1.0f, -9.0f, -8.0f,
     1.0f,  7.0f,  8.0f, -9.0f
};

/* Result of differentiation B*p with respect to B, where p is a quaternion */
static const float buffs_BpDiffB[] = {
     69.3764f, -18.6179f, -78.7688f,  90.3680f,
     18.6179f,  69.3764f, -90.3680f, -78.7688f,
     78.7688f,  90.3680f,  69.3764f,  18.6179f,
    -90.3680f,  78.7688f, -18.6179f,  69.3764f
};

/* Result of differentiation QA*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_QAvDiffV[] = {
    0.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

/* Result of differentiation QI*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_QIvDiffV[] = {
    -1.0f, 0.0f,  0.0f,
     0.0f, 0.0f,  0.0f,
     0.0f, 0.0f, -1.0f,
     0.0f, 1.0f,  0.0f
};

/* Result of differentiation QJ*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_QJvDiffV[] = {
     0.0f, -1.0f, 0.0f,
     0.0f,  0.0f, 1.0f,
     0.0f,  0.0f, 0.0f,
    -1.0f,  0.0f, 0.0f
};

/* Result of differentiation QK*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_QKvDiffV[] = {
    0.0f,  0.0f, -1.0f,
    0.0f, -1.0f,  0.0f,
    1.0f,  0.0f,  0.0f,
    0.0f,  0.0f,  0.0f
};

/* Result of differentiation A*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_AvDiffV[] = {
     8.0f, -7.0f, -1.0f,
    -9.0f, -1.0f,  7.0f,
     1.0f, -9.0f,  8.0f,
    -7.0f, -8.0f, -9.0f
};

/* Result of differentiation B*v with respect to v, where v is a pure quaternion (quaternionized vector) */
static const float buffs_BvDiffV[] = {
    -18.6179f, -78.7688f,  90.3680f,
     69.3764f,  90.3680f,  78.7688f,
    -90.3680f,  69.3764f, -18.6179f,
    -78.7688f,  18.6179f,  69.3764f
};

/* clang-format on */

#endif