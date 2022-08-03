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

#include "quat.h"


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


void quat_quat2euler(quat_t *q, float *roll, float *pitch, float *yaw)
{
	/* FIXME: is this normalization needed? */
	quat_normalize(q);
	*roll = atan2(2 * (q->a * q->i + q->j * q->k), 1 - 2 * (q->i * q->i + q->j * q->j));
	*pitch = asin(2 * (q->a * q->j - q->k * q->i));
	*yaw = atan2(2 * (q->a * q->k + q->i * q->j), 1 - 2 * (q->j * q->j + q->k * q->k));
}
