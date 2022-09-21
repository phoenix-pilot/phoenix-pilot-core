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

#include <stdio.h>
#include <math.h>

#include <vec.h>

#include "quat.h"


void quat_sum(const quat_t *A, const quat_t *B, quat_t *C)
{
	C->a = A->a + B->a;
	C->i = A->i + B->i;
	C->j = A->j + B->j;
	C->k = A->k + B->k;
}


void quat_add(quat_t *A, const quat_t *B)
{
	A->a += B->a;
	A->i += B->i;
	A->j += B->j;
	A->k += B->k;
}


void quat_dif(const quat_t *A, const quat_t *B, quat_t *C)
{
	C->a = A->a - B->a;
	C->i = A->i - B->i;
	C->j = A->j - B->j;
	C->k = A->k - B->k;
}


void quat_sub(quat_t *A, const quat_t *B)
{
	A->a -= B->a;
	A->i -= B->i;
	A->j -= B->j;
	A->k -= B->k;
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
	// return (quat_t){.a = A->a, .i = -A->i, .j = -A->j, .k = -A->k};
}


void quat_sandwich(const quat_t *A, const quat_t *B, quat_t *C)
{
	const quat_t Acjg = { .a = A->a, .i = -A->i, .j = -A->j, .k = -A->k };
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


void quat_quat2euler(const quat_t *q, float *roll, float *pitch, float *yaw)
{
	quat_t tmp = *q;

	/* FIXME: is this normalization needed? */
	quat_normalize(&tmp);
	*roll = atan2(2 * (tmp.a * tmp.i + tmp.j * tmp.k), 1 - 2 * (tmp.i * tmp.i + tmp.j * tmp.j));
	*pitch = asin(2 * (tmp.a * tmp.j - tmp.k * tmp.i));
	*yaw = atan2(2 * (tmp.a * tmp.k + tmp.i * tmp.j), 1 - 2 * (tmp.j * tmp.j + tmp.k * tmp.k));
}


void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q)
{
	float a = vec_dot(v1, v2);
	quat_t qv1, qv2;
	/*vec_t tmp;*/

	if (a > 0.999999f) {
		/* if vectors are close to parallel */
		quat_idenWrite(q);
		return;
	}
	if (a < -0.999999f) {
		/* if vectors are close to antiparallel */
		/* calculating quaternion, which rotates 180 degrees along axis perpendicular to `v1` and `v2` */
		vec_normal(v1, v2, (vec_t *)q);
		q->a = 0;
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
	quat_t *v;

	/* cast vector as quaternion */
	v = (quat_t *)vec;
	v->a = 0;

	quat_sandwichFast(qRot, v, v);
	v->a = 0;
}


void quat_rotQuat(const vec_t *axis, float angle, quat_t *q)
{
	float coeff, len;

	len = vec_len(axis);

	if (len == 0) {
		quat_idenWrite(q);
		return;
	}

	coeff = sinf(angle / 2) / len;

	q->a = cosf(angle / 2);
	q->i = axis->x * coeff;
	q->j = axis->y * coeff;
	q->k = axis->z * coeff;
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

	/* use 0 as NULL to not include stdio just for that */
	if (help_q != 0 && quat_dot(res, help_q) < 0) {
		quat_times(res, -1);
	}
}
