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

#ifndef PHOENIX_QVDIFF_H_
#define PHOENIX_QVDIFF_H_


#include <matrix.h>
#include <vec.h>
#include <quat.h>


/* Calculates derivative: d(q * p *cjg(q)) / dq with assumptions:
 * 1) `out` is 3x4 matrix
 * 2) `q` is rotation quaternion
 * 3) `p` is vector (or pure quaternion vectorial part)
 */
int qvdiff_qvqDiffQ(const quat_t *q, const vec_t *v, matrix_t *out);


/* Calculates derivative: d(cjg(q) * p *q) / dq with assumptions:
 * 1) `out` is 3x4 matrix
 * 2) `q` is rotation quaternion
 * 3) `p` is vector (or pure quaternion vectorial part)
 */
int qvdiff_cqvqDiffQ(const quat_t *q, const vec_t *v, matrix_t *out);


/* Calculates derivative: d(q * v * cjg(q)) / d(v) with assumptions:
 * 1) `out` is 3x3 matrix
 * 2) q is quaternion
 * Note: this derivative does not need value of `v`
 */
int qvdiff_qvqDiffV(const quat_t *q, matrix_t *out);


/* Calculates derivative: d(q * p) / d(q) with assumptions:
 * 1) `out` is 4x4 matrix
 * 2) 'q' and 'p' are quaternions
 */
int qvdiff_qpDiffQ(const quat_t *p, matrix_t *out);


/* Calculates derivative: d(q * p) / d(q) with assumptions:
 * 1) `out` is 4x3 matrix
 * 2) 'q' is a quaternion
 * 3) 'p' is a quaternionized vector (only imaginary terms are significant)
 */
int qvdiff_qpDiffP(const quat_t *q, matrix_t *out);

#endif
