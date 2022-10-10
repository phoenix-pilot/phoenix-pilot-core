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


/* Compares dwo quaternions. Returns 0 if they are equal and not 0 if they are different */
static inline int quat_cmp(const quat_t *A, const quat_t *B)
{
	return !(A->a == B->a && A->i == B->i && A->j == B->j && A->k == B->k);
}


/* Calculates length of quaternion A */
extern float quat_len(const quat_t *A);


/* C = A + B; stores sum of A and B into C */
extern void quat_sum(const quat_t *A, const quat_t *B, quat_t *C);


/* A += B; adds quaternion B to A */
extern void quat_add(quat_t *A, const quat_t *B);


/* C = A - B; stores difference of A and B in C */
extern void quat_dif(const quat_t *A, const quat_t *B, quat_t *C);


/* A -= B; subtracts B from A */
extern void quat_sub(quat_t *A, const quat_t *B);


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


/* Calculates rotation euler angles from rotation quaternion. Hierarchy of angels are yaw, pitch, roll (ZYX) */
extern void quat_quat2euler(const quat_t *q, float *roll, float *pitch, float *yaw);


/* calculate quaternion q that rotates v1 into v2 along axis perpendicular to `v1` and `v2`. Both vectors need to be unit vectors */
extern void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q);


/* rotates vector using rotation quaternion */
extern void quat_vecRot(vec_t *vec, const quat_t *qRot);


/* calculate quaternion, which rotates about `angle` in radians along `axis` */
extern void quat_rotQuat(const vec_t *axis, float angle, quat_t *q);


/* calculates quaternion res (closest to help_q), that rotates frame of reference (v1, v2) into (w1, w2) */
extern void quat_frameRot(const vec_t *v1, const vec_t *v2, const vec_t *w1, const vec_t *w2, quat_t *res, const quat_t *help_q);


#endif
