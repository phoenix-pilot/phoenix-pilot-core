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


vec_t vec_sub(vec_t *A, vec_t *B)
{
	return (vec_t){.x = A->x - B->x, .y = A->y - B->y, .z = A->z - B->z};
}


vec_t vec_cross(vec_t *A, vec_t *B)
{
	return (vec_t){.x = A->y * B->z - A->z * B->y, .y = A->z * B->x - A->x * B->z, .z = A->x * B->y - A->y * B->x};
}


float vec_dot(vec_t *A, vec_t *B)
{
	return A->x * B->x + A->y * B->y + A->z * B->z;
}


vec_t vec_times(vec_t *A, float a)
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
	return vec_times(&cross, 1./vec_len(&cross));
}


vec_t * vec_normalize(vec_t * A)
{
	float len = A->x * A->x + A->y * A->y + A->z * A->z;
	len = sqrt(len); /* FIXME: fast iverse square needed here! */

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

float quat_dot(const quat_t *A, const quat_t *B)
{
	return A->a * B->a + A->i * B->i + A->j * B->j + A->k * B->k; 
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
	float norm = 1 / sqrt(A->a * A->a + A->i * A->i + A->j * A->j + A->k * A->k); /* FIXME: fast iverse square needed here! */
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


quat_t quat_uvec2uvec(vec_t *v1, vec_t *v2)
{
	quat_t q;
	double a = vec_dot(v1, v2);

	if (a > 0.99999999) {
		/* if vectors are close to parallel */
		return IDEN_QUAT;
	}
	if (a < -0.99999999) {
		/* if vectors are close to antiparallel */
		return PI_QUAT;
	}

	v1->l = v2->l = 0;
	q = quat_mlt((quat_t*)v1, (quat_t*)v2);
	q.a *= -1;
	a = q.a;
	q.a += 1;
	quat_times(&q, (1.)/sqrt(2 + 2 * a));
	quat_normalize(&q);

	return q;
}


vec_t * quat_vecrot(vec_t * vec, quat_t * rotquat)
{
	quat_t * v; 

	/* cast vector as quaternion */
	v = (quat_t*)vec;
	v->a = 0;

	*v = quat_sandwich(rotquat, v);
	v->a = 0;

	return vec;
}


quat_t quat_framerot(vec_t *v1, vec_t *v2, vec_t *w1, vec_t *w2, quat_t * help_q)
{
	vec_t n, p;
	quat_t q1, q2;

	n = vec_cross(v1, v2);
	p = vec_cross(w1, w2);
	vec_normalize(&n);
	vec_normalize(&p);

	q1 = quat_uvec2uvec(v1, w1);
	q2 = quat_uvec2uvec(quat_vecrot(&n, &q1), &p);
	q2 = quat_mlt(&q2, &q1);
	quat_normalize(&q2); /* FIXME: is this normalization necessary? */

	if (help_q != NULL && quat_dot(&q2, help_q) < 0) {
		quat_times(&q2, -1);
	}
	return q2;
}


vec_t quat_quat2euler(quat_t q)
{
	return vec(
		atan2(2 * (q.a * q.i + q.j * q.k), 1 - 2 * (q.i * q.i + q.j * q.j)),
		asin(2 * (q.a * q.j - q.k * q.i)),
		atan2(2 * (q.a * q.k + q.i * q.j), 1 - 2 * (q.j * q.j + q.k * q.k))
	);
}
