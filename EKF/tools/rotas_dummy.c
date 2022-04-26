/*
 * Phoenix-Pilot
 *
 * rotas
 *
 * quaternion and 3d vectors operations library
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

#include "rotas_dummy.h"


/* ==================== */
/*   VECTOR FUNCTIONS   */
/* ==================== */


vec_t vec(float x, float y, float z)
{
	return (vec_t){.x = x, .y = y, .z = z};
}


vec_t vec_add(vec_t *A, vec_t *B)
{
	return (vec_t){.x = A->x + B->x, .y = A->y + B->y, .z = A->z + B->z};
}


vec_t vec_cross(vec_t *A, vec_t *B)
{
	return (vec_t){.x = A->y * B->z - A->z * B->y, .y = A->z * B->x - A->x * B->z, .z = A->x * B->y - A->y * B->x};
}


float vec_dot(vec_t *A, vec_t *B)
{
	return A->x * B->x + A->y * B->y + A->z * B->z;
}


vec_t vec_scl(vec_t *A, float a)
{
	return (vec_t){.x = A->x * a, .y = A->y * a, .z = A->z * a};
}


float vec_len(vec_t *A)
{
	return sqrt(A->x * A->x + A->y * A->y + A->z * A->z);
}


vec_t vec_normal(vec_t *A, vec_t *B)
{
	vec_t cross = vec_cross(A, B);
	return vec_scl(&cross, 1./vec_len(&cross));
}


vec_t * vec_normalize(vec_t * A)
{
	float len = A->x * A->x + A->y * A->y + A->z * A->z;
	len = sqrt(len);

	A->x /= len;
	A->y /= len;
	A->z /= len;

	return A;
}


/* ==================== */
/* QUATERNION FUNCTIONS */
/* ==================== */


quat_t quat(float a, float i, float j, float k)
{
	return (quat_t){.a = a, .i = i, .j = j, .k = k};
}


quat_t quat_add(const quat_t *A, const quat_t *B)
{
	return (quat_t){.a = A->a + B->a, .i = A->i + B->i, .j = A->j + B->j, .k = A->k + B->k};
}


quat_t quat_mlt(const quat_t *A, const quat_t *B)
{
	return (quat_t){
		.a = A->a * B->a - A->i * B->i - A->j * B->j - A->k * B->k,
		.i = A->a * B->i + A->i * B->a + A->j * B->k - A->k * B->j,
		.j = A->a * B->j - A->i * B->k + A->j * B->a + A->k * B->i,
		.k = A->a * B->k + A->i * B->j - A->j * B->i + A->k * B->a
	};
}


quat_t quat_cjg(const quat_t *A)
{
	return (quat_t){.a = A->a, .i = -A->i, .j = -A->j, .k = -A->k};
}


quat_t quat_sandwich(const quat_t *A, const quat_t *B)
{
	quat_t AB = quat_mlt(A, B), Acjg = quat_cjg(A);
	return quat_mlt(&AB, &Acjg);
}


quat_t quat_sandwich_fast(const quat_t *A, const quat_t *B)
{
	float arg[4] = {
		(A->a * B->a - A->i * B->i - A->j * B->j - A->k * B->k),
		(A->a * B->i + A->i * B->a + A->j * B->k - A->k * B->j),
		(A->a * B->j - A->i * B->k + A->j * B->a + A->k * B->i),
		(A->a * B->k + A->i * B->j - A->j * B->i + A->k * B->a)};

	return (quat_t){
		.a =  arg[0] * A->a + arg[1] * A->i + arg[2] * A->j + arg[3] * A->k,
		.i = -arg[0] * A->i + arg[1] * A->a - arg[2] * A->k + arg[3] * A->j,
		.j = -arg[0] * A->j + arg[1] * A->k + arg[2] * A->a - arg[3] * A->i,
		.k = -arg[0] * A->k - arg[1] * A->j + arg[2] * A->i + arg[3] * A->a
	};
}


void quat_print(const quat_t *A)
{
	printf("%f %+fi %+fj %+fk\n", A->a, A->i, A->j, A->k);
}


void quat_normalize(quat_t * A)
{
	float norm = 1 / sqrt(A->a * A->a + A->i * A->i + A->j * A->j + A->k * A->k);
	A->a *= norm;
	A->i *= norm;
	A->j *= norm;
	A->k *= norm;
}

quat_t * quat_times(quat_t * A, float x)
{
	A->a *= x;
	A->i *= x;
	A->j *= x;
	A->k *= x;
	return A;
}


/* ==================== */
/* ==== TRANSFORMS ==== */
/* ==================== */


quat_t quat_vec2vec(vec_t *AA, vec_t * BB)
{
	vec_t A = *AA;
	vec_t B = *BB;

	vec_normalize(&A);
	vec_normalize(&B);

	vec_t cross = vec_cross(&A, &B);
	float dot = vec_dot(&A, &B);
	float len = sqrt(dot * dot + cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);

	/* handle edge case if A ~== B */
	if (dot > 0.999999999) {
		return IDEN_QUAT;
	}

	/* already embeding identity quaternion (1 + 0i + 0j + 0k) to get half rotation quaternion */
	quat_t q = {.a = 1 + dot / len, .i =  cross.x / len, .j = cross.y / len, .k = cross.z / len};

	quat_normalize(&q);

	return q;
}


vec_t * quat_vecrot(vec_t * vec, quat_t * rotquat)
{
	quat_t * v; 

	/* cast vector as quaternion */
	v = (quat_t*)vec;
	v->a = 0;

	*v = quat_sandwich_fast(rotquat, v);
	v->a = 0;

	return vec;
}

quat_t quat_framerot(vec_t v1, vec_t v2, vec_t w1, vec_t w2)
{
	quat_t q1 = quat_vec2vec(&v1, &w1), q2;
	
	quat_vecrot(&v2, &q1);
	q2 = quat_vec2vec(&v2, &w2);

	return quat_mlt(&q2, &q1);
}

vec_t quat_quat2euler(quat_t q)
{
	return vec(
		atan2(2*(q.a * q.i + q.j * q.k), 1 - 2 * (q.i * q.i + q.j * q.j)),
		asin(2 * (q.a * q.j - q.k * q.i)),
		atan2(2 * (q.a * q.k + q.i * q.j), 1 - 2 * (q.j * q.j + q.k * q.k))
	);
}
