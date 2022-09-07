/*
 * Phoenix-Pilot
 *
 * vec - 3d vectors operations library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PHOENIX_VEC_H__
#define __PHOENIX_VEC_H__


/* vector has additional value 'l' that is only for easy casting to/from quaternion (quat_t <-> vec_t) */
typedef struct {
	float x, y, z, l;
} vec_t;


/* Compares vectors. Return 0 if A is equal to B and not 0 in other case */
static inline int vec_cmp(const vec_t *A, const vec_t *B)
{
	return !(A->x == B->x && A->y == B->y && A->z == B->z);
}


/* C = A + B; stores sum of A and B into C */
extern void vec_sum(const vec_t *A, const vec_t *B, vec_t *C);


/* A += B; adds B to A */
extern void vec_add(vec_t *A, const vec_t *B);


/* C = A - B; stores difference of A and B in C */
extern void vec_dif(const vec_t *A, const vec_t *B, vec_t *C);


/* A -= B; subtracts B from A */
extern void vec_sub(vec_t *A, const vec_t *B);


/* C = A x B; stores cross product of A and B into C */
extern void vec_cross(const vec_t *A, const vec_t *B, vec_t *C);


/* returns the dot product of two vectors */
extern float vec_dot(const vec_t *A, const vec_t *B);


/* multiplies each vector element times scalar 'a' */
extern void vec_times(vec_t *A, float a);


/* returns length of passed vector */
extern float vec_len(const vec_t *A);


/* calculates unit vector C that is perpendicular to both A and B */
extern void vec_normal(const vec_t *A, const vec_t *B, vec_t *C);


/* normalizes A to a unit vector */
extern void vec_normalize(vec_t *A);

#endif
