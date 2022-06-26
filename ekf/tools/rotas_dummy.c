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


/* ==================== */
/* QUATERNION FUNCTIONS */
/* ==================== */


quat_t quat(float a, float i, float j, float k)
{
	return (quat_t){.a = a, .i = i, .j = j, .k = k};
}


void quat_add(quat_t *A, const quat_t *B)
{
	A->a += B->a;
	A->i += B->i;
	A->j += B->j;
	A->k += B->k;
}


void quat_mlt(const quat_t *A, const quat_t *B, quat_t *C)
{
	C->a = A->a * B->a - A->i * B->i - A->j * B->j - A->k * B->k;
	C->i = A->a * B->i + A->i * B->a + A->j * B->k - A->k * B->j;
	C->j = A->a * B->j - A->i * B->k + A->j * B->a + A->k * B->i;
	C->k = A->a * B->k + A->i * B->j - A->j * B->i + A->k * B->a;
}

float quat_dot(const quat_t *A, const quat_t *B)
{
	return A->a * B->a + A->i * B->i + A->j * B->j + A->k * B->k; 
}


void quat_cjg(quat_t *A)
{
	A->i = -A->i;
	A->j = -A->j;
	A->k = -A->k;
	//return (quat_t){.a = A->a, .i = -A->i, .j = -A->j, .k = -A->k};
}


void quat_sandwich(const quat_t *A, const quat_t *B, quat_t *C)
{
	const quat_t Acjg = { .a = A->i, .i = -A->i, .j = -A->j, .k = -A->k };
	quat_t AB;

	quat_mlt(A, B, &AB);
	quat_mlt(&AB, &Acjg, C);
}


void quat_sandwichFast(const quat_t *A, const quat_t *B, quat_t *C)
{
	float arg[4] = {
		(A->a * B->a - A->i * B->i - A->j * B->j - A->k * B->k),
		(A->a * B->i + A->i * B->a + A->j * B->k - A->k * B->j),
		(A->a * B->j - A->i * B->k + A->j * B->a + A->k * B->i),
		(A->a * B->k + A->i * B->j - A->j * B->i + A->k * B->a)
	};

	C->a = arg[0] * A->a + arg[1] * A->i + arg[2] * A->j + arg[3] * A->k;
	C->i = -arg[0] * A->i + arg[1] * A->a - arg[2] * A->k + arg[3] * A->j;
	C->j = -arg[0] * A->j + arg[1] * A->k + arg[2] * A->a - arg[3] * A->i;
	C->k = -arg[0] * A->k - arg[1] * A->j + arg[2] * A->i + arg[3] * A->a;
}


void quat_print(const quat_t *A)
{
	printf("%f %+fi %+fj %+fk\n", A->a, A->i, A->j, A->k);
}


void quat_normalize(quat_t *A)
{
	float norm = 1 / sqrt(A->a * A->a + A->i * A->i + A->j * A->j + A->k * A->k); /* FIXME: fast iverse square needed here! */
	A->a *= norm;
	A->i *= norm;
	A->j *= norm;
	A->k *= norm;
}

void quat_times(quat_t *A, float x)
{
	A->a *= x;
	A->i *= x;
	A->j *= x;
	A->k *= x;
}


/* ==================== */
/* ==== TRANSFORMS ==== */
/* ==================== */


void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q)
{
	float a = vec_dot(v1, v2);
	quat_t qv1, qv2;

	if (a > 0.99999999) {
		/* if vectors are close to parallel */
		*q = IDEN_QUAT;
		return;
	}
	if (a < -0.99999999) {
		/* if vectors are close to antiparallel */
		*q = PI_QUAT;
		return;
	}

	qv1 = *(quat_t *)v1;
	qv2 = *(quat_t *)v2;
	qv1.a = qv2.a = 0;

	quat_mlt(&qv1, &qv2, q);
	q->a *= -1;
	a = q->a;
	q->a += 1;
	quat_times(q, (1.) / sqrt(2 + 2 * a));
	quat_normalize(q);
}


void quat_vecRot(vec_t *vec, const quat_t *qRot)
{
	quat_t * v; 

	/* cast vector as quaternion */
	v = (quat_t*)vec;
	v->a = 0;

	quat_sandwichFast(qRot, v, v);
	v->a = 0;
}


void quat_frameRot(const vec_t *v1, const vec_t *v2, const vec_t *w1, const vec_t *w2, quat_t *res, const quat_t *help_q)
{
	vec_t n, p;
	quat_t q1, q2;

	vec_cross(v1, v2, &n);
	vec_cross(w1, w2, &p);
	vec_normalize(&n);
	vec_normalize(&p);

	quat_uvec2uvec(v1, w1, &q1);
	quat_vecRot(&n, &q1);
	quat_uvec2uvec(&n, &p, &q2);
	quat_mlt(&q2, &q1, res);
	quat_normalize(res); /* FIXME: is this normalization necessary? */

	if (help_q != NULL && quat_dot(res, help_q) < 0) {
		quat_times(res, -1);
	}
}


void quat_quat2euler(quat_t *q, float *roll, float *pitch, float *yaw)
{
	/* FIXME: is this normalization needed? */
	quat_normalize(q);
	*roll = atan2(2 * (q->a * q->i + q->j * q->k), 1 - 2 * (q->i * q->i + q->j * q->j));
	*pitch = asin(2 * (q->a * q->j - q->k * q->i));
	*yaw = atan2(2 * (q->a * q->k + q->i * q->j), 1 - 2 * (q->j * q->j + q->k * q->k));
}
