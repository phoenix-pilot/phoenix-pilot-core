/*
 * Phoenix-Pilot
 *
 * qdiff - quaternion-vector operations derivatives
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <vec.h>
#include <matrix.h>
#include <quat.h>


/* Calculates cross product matrix for vector v so that when left-hand-multiplied by a vector p produces cross product (v x p).
 * `out` is assumed to be 3x3, untransposed matrix.
 */
static void qvdiff_crossMat(const vec_t *v, matrix_t *out)
{
	/* diagonal */
	MATRIX_DATA(out, 0, 0) = 0;
	MATRIX_DATA(out, 1, 1) = 0;
	MATRIX_DATA(out, 2, 2) = 0;

	/* upper/lower triangles */
	MATRIX_DATA(out, 2, 1) = v->x;
	MATRIX_DATA(out, 1, 2) = -v->x;
	MATRIX_DATA(out, 0, 2) = v->y;
	MATRIX_DATA(out, 2, 0) = -v->y;
	MATRIX_DATA(out, 1, 0) = v->z;
	MATRIX_DATA(out, 0, 1) = -v->z;
}


int qvdiff_qvqDiffQ(const quat_t *q, const vec_t *v, matrix_t *out)
{
	float val1, val2, val3, val4;

	if (matrix_rowsGet(out) != 3 || matrix_colsGet(out) != 4) {
		return -1;
	}

	if (out->transposed != 0) {
		out->transposed = 0;
		out->cols = 4;
		out->rows = 3;
	}

	val1 = 2 * (q->a * v->x + q->j * v->z - q->k * v->y);
	val2 = 2 * (q->a * v->y + q->k * v->x - q->i * v->z);
	val3 = 2 * (q->a * v->z + q->i * v->y - q->j * v->x);
	val4 = 2 * (q->i * v->x + q->j * v->y + q->k * v->z);

	MATRIX_DATA(out, 0, 0) = val1;
	MATRIX_DATA(out, 1, 0) = val2;
	MATRIX_DATA(out, 2, 0) = val3;

	MATRIX_DATA(out, 0, 1) = val4;
	MATRIX_DATA(out, 1, 1) = -val3;
	MATRIX_DATA(out, 2, 1) = val2;

	MATRIX_DATA(out, 0, 2) = val3;
	MATRIX_DATA(out, 1, 2) = val4;
	MATRIX_DATA(out, 2, 2) = -val1;

	MATRIX_DATA(out, 0, 3) = -val2;
	MATRIX_DATA(out, 1, 3) = val1;
	MATRIX_DATA(out, 2, 3) = val4;

	return 0;
}


int qvdiff_qvqDiffV(const quat_t *q, matrix_t *out)
{
	float qqtData[3 * 3] = { 0 }, tmp;
	matrix_t qqt = { .data = qqtData, .rows = 3, .cols = 3, .transposed = 0 };
	const vec_t qVec = { .x = q->i, .y = q->j, .z = q->k };

	if (matrix_rowsGet(out) != 3 || matrix_colsGet(out) != 3) {
		return -1;
	}

	out->transposed = 0;

	qvdiff_crossMat(&qVec, out);
	matrix_times(out, 2 * q->a);
	/* Diagonal terms of `out` are now set to zeroes by `qvdiff_crossMat`. Proceed with diagonal terms */
	tmp = q->a * q->a - (q->i * q->i + q->j * q->j + q->k * q->k);
	MATRIX_DATA(out, 0, 0) = tmp;
	MATRIX_DATA(out, 1, 1) = tmp;
	MATRIX_DATA(out, 2, 2) = tmp;

	MATRIX_DATA(&qqt, 0, 0) = q->i * q->i;
	MATRIX_DATA(&qqt, 1, 1) = q->j * q->j;
	MATRIX_DATA(&qqt, 2, 2) = q->k * q->k;

	tmp = q->i * q->j;
	MATRIX_DATA(&qqt, 0, 1) = tmp;
	MATRIX_DATA(&qqt, 1, 0) = tmp;

	tmp = q->i * q->k;
	MATRIX_DATA(&qqt, 0, 2) = tmp;
	MATRIX_DATA(&qqt, 2, 0) = tmp;

	tmp = q->j * q->k;
	MATRIX_DATA(&qqt, 1, 2) = tmp;
	MATRIX_DATA(&qqt, 2, 1) = tmp;

	matrix_times(&qqt, 2);
	matrix_add(out, &qqt, NULL);

	return 0;
}


int qvdiff_qpDiffQ(const quat_t *p, matrix_t *out)
{
	if (out->cols != 4 || out->rows != 4) {
		return -1;
	}

	out->transposed = 0;

	MATRIX_DATA(out, 0, 0) = p->a;
	MATRIX_DATA(out, 1, 1) = p->a;
	MATRIX_DATA(out, 2, 2) = p->a;
	MATRIX_DATA(out, 3, 3) = p->a;

	MATRIX_DATA(out, 1, 0) = p->i;
	MATRIX_DATA(out, 2, 3) = p->i;
	MATRIX_DATA(out, 0, 1) = -p->i;
	MATRIX_DATA(out, 3, 2) = -p->i;

	MATRIX_DATA(out, 2, 0) = p->j;
	MATRIX_DATA(out, 3, 1) = p->j;
	MATRIX_DATA(out, 0, 2) = -p->j;
	MATRIX_DATA(out, 1, 3) = -p->j;

	MATRIX_DATA(out, 1, 2) = p->k;
	MATRIX_DATA(out, 3, 0) = p->k;
	MATRIX_DATA(out, 0, 3) = -p->k;
	MATRIX_DATA(out, 2, 1) = -p->k;

	return 0;
}


int qvdiff_qpDiffP(const quat_t *q, matrix_t *out)
{
	if (matrix_rowsGet(out) != 4 || matrix_colsGet(out) != 3) {
		return -1;
	}

	if (out->transposed != 0) {
		out->transposed = 0;
		out->cols = 3;
		out->rows = 4;
	}

	MATRIX_DATA(out, 1, 0) = q->a;
	MATRIX_DATA(out, 2, 1) = q->a;
	MATRIX_DATA(out, 3, 2) = q->a;

	MATRIX_DATA(out, 3, 1) = q->i;
	MATRIX_DATA(out, 0, 0) = -q->i;
	MATRIX_DATA(out, 2, 2) = -q->i;

	MATRIX_DATA(out, 1, 2) = q->j;
	MATRIX_DATA(out, 0, 1) = -q->j;
	MATRIX_DATA(out, 3, 0) = -q->j;

	MATRIX_DATA(out, 2, 0) = q->k;
	MATRIX_DATA(out, 0, 2) = -q->k;
	MATRIX_DATA(out, 1, 1) = -q->k;

	return 0;
}
