/*
 * Phoenix-Pilot
 *
 * qrot - quaternion/vector rotation library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <vec.h>
#include <quat.h>
#include <math.h>


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

	/* use 0 as NULL to not include stdio just for that */
	if (help_q != 0 && quat_dot(res, help_q) < 0) {
		quat_times(res, -1);
	}
}
