/*
 * Phoenix-Pilot
 *
 * quat - quaternion algebra library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PHOENIX_QUAT_H__
#define __PHOENIX_QUAT_H__

#include <vec.h>

typedef struct {
	float i, j, k, a;
} quat_t;


/* q = {1, 0, 0, 0}; makes quaternion pointed by `q` an identity quaternion */
static inline void quat_idenWrite(quat_t *q)
{
	q->i = q->j = q->k = 0;
	q->a = 1;
}

/* q = {0, 1, 0, 0}; makes quaternion pointed by `q` a PI angled rotation quaternion */
static inline void quat_piWrite(quat_t *q)
{
	q->a = q->j = q->k = 0;
	q->i = 1;
}


/* A += B; adds quaternion B to A */
extern void quat_add(quat_t *A, const quat_t *B);


/* A * B = C; multiply A and B and store result in C */
extern void quat_mlt(const quat_t *A, const quat_t *B, quat_t *C);


/* x = AB | returns euclidean 4D dot product of two quaternions */
extern float quat_dot(const quat_t *A, const quat_t *B);


/* B = A* | conjugates quaternion A */
extern void quat_cjg(quat_t *A);


/* calculates C that is result of: A * B * cjg(A), C must not overlap with A or B */
extern void quat_sandwich(const quat_t *A, const quat_t *B, quat_t *C);


/* print quaternion in human readable form */
extern void quat_print(const quat_t *A);


/* C = C / ||C|| | normalizes quaternion to a unit quaternion */
extern void quat_normalize(quat_t *A);


/* calculates C that is result of: A * B * cjg(A), C can overlap with A or B */
extern void quat_sandwichFast(const quat_t *A, const quat_t *B, quat_t *C);


/* multiplies each quaternion element by a factor of x */
extern void quat_times(quat_t *A, float x);


/* calculates rotation euler angles from rotation quaternion */
extern void quat_quat2euler(quat_t *q, float *roll, float *pitch, float *yaw);


/* calculate quaternion q that rotates v1 into v2, assumed len(v1) == len(v2) */
extern void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q);


/* rotates vector using rotation quaternion */
extern void quat_vecRot(vec_t *vec, const quat_t *qRot);


/* calculates quaternion res (closest to help_q), that rotates frame of reference (v1, v2) into (w1, w2) */
extern void quat_frameRot(const vec_t *v1, const vec_t *v2, const vec_t *w1, const vec_t *w2, quat_t *res, const quat_t *help_q);


#endif
