/*
 * Phoenix-Pilot
 *
 * lma.h usage example
 *
 * Function: x_1 = p_0 * sqrt( x_0 - p_1 * (x_0)^2 + p_2 * (x_0)^3 )
 * Sample data generated for:
 * p: (0.5, 0.5, 0.1)
 * x1: in range [0.2, 4.0] with a step of 0.2
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <matrix.h>
#include <lma.h>


#define SAMPLES 20
#define STEPS   16


/* Sample data */
const float tabX0[SAMPLES] = { 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40, 1.60, 1.80, 2.00, 2.20, 2.40, 2.60, 2.80, 3.00, 3.20, 3.40, 3.60, 3.80, 4.00 };
const float tabX1[SAMPLES] = { 0.21, 0.29, 0.33, 0.36, 0.39, 0.40, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.49, 0.52, 0.55, 0.58, 0.62, 0.67, 0.72, 0.77 };


int jacobian(const matrix_t *P, const matrix_t *V, matrix_t *J, bool log)
{
	float p[] = { MATRIX_DATA(P, 0, 0), MATRIX_DATA(P, 0, 1), MATRIX_DATA(P, 0, 2) };
	float x = MATRIX_DATA(V, 0, 0);
	float tmp;

	tmp = sqrt(x * (-p[1] * x + p[2] * x * x + 1));

	MATRIX_DATA(J, 0, 0) = sqrt(x - p[1] * x * x + p[2] * x * x * x);
	MATRIX_DATA(J, 0, 1) = -0.5 * p[0] * x * x / tmp;
	MATRIX_DATA(J, 0, 2) = 0.5 * p[0] * x * x * x / tmp;

	return 0;
}


int residuum(const matrix_t *P, const matrix_t *V, float *res, bool log)
{
	float p[] = { MATRIX_DATA(P, 0, 0), MATRIX_DATA(P, 0, 1), MATRIX_DATA(P, 0, 2) };
	float x[] = { MATRIX_DATA(V, 0, 0), MATRIX_DATA(V, 0, 1) };

	*res = p[0] * sqrt(x[0] - p[1] * pow(x[0], 2) + p[2] * pow(x[0], 3)) - x[1];

	return 0;
}

void guess(matrix_t *P)
{
	MATRIX_DATA(P, 0, 0) = 0;
	MATRIX_DATA(P, 0, 1) = 0;
	MATRIX_DATA(P, 0, 2) = 0;
}


int main(int argc, char **argv)
{
	fit_lma_t lma;
	int i;

	lma_init(2, 3, SAMPLES, jacobian, residuum, guess, &lma);

	for (i = 0; i < SAMPLES; i++) {
		MATRIX_DATA(&lma.samples, i, 0) = tabX0[i];
		MATRIX_DATA(&lma.samples, i, 1) = tabX1[i];
	}

	lma_fit(STEPS, &lma, LMALOG_NONE);

	printf(
		"%f %f %f\n",
		MATRIX_DATA(&lma.paramsVec, 0, 0),
		MATRIX_DATA(&lma.paramsVec, 0, 1),
		MATRIX_DATA(&lma.paramsVec, 0, 2));

	lma_done(&lma);
}
