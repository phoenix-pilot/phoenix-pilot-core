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

#include <stdio.h>
#include <math.h>

#include "vec.h"


vec_t vec(float x, float y, float z)
{
	return (vec_t) { .x = x, .y = y, .z = z };
}


void vec_sum(const vec_t *A, const vec_t *B, vec_t *C)
{
	C->x = A->x + B->x;
	C->y = A->y + B->y;
	C->z = A->z + B->z;
}


void vec_add(vec_t *A, const vec_t *B)
{
	A->x += B->x;
	A->y += B->y;
	A->z += B->z;
}


void vec_dif(const vec_t *A, const vec_t *B, vec_t *C)
{
	C->x = A->x - B->x;
	C->y = A->y - B->y;
	C->z = A->z - B->z;
}


void vec_sub(vec_t *A, const vec_t *B)
{
	A->x -= B->x;
	A->y -= B->y;
	A->z -= B->z;
}


void vec_cross(const vec_t *A, const vec_t *B, vec_t *C)
{
	C->x = A->y * B->z - A->z * B->y;
	C->y = A->z * B->x - A->x * B->z;
	C->z = A->x * B->y - A->y * B->x;
}


float vec_dot(const vec_t *A, const vec_t *B)
{
	return A->x * B->x + A->y * B->y + A->z * B->z;
}


void vec_times(vec_t *A, float a)
{
	A->x *= a;
	A->y *= a;
	A->z *= a;
}


float vec_len(const vec_t *A)
{
	return sqrt(A->x * A->x + A->y * A->y + A->z * A->z);
}


void vec_normal(const vec_t *A, const vec_t *B, vec_t *C)
{
	float len;

	vec_cross(A, B, C);
	len = vec_len(C);
	(len != 0) ? vec_times(C, 1.F / len) : vec_times(C, 0);
}


void vec_normalize(vec_t *A)
{
	float len = A->x * A->x + A->y * A->y + A->z * A->z;
	len = sqrt(len); /* FIXME: fast iverse square needed here! */

	A->x /= len;
	A->y /= len;
	A->z /= len;
}
