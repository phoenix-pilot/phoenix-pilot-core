/*
 * Phoenix-Pilot
 *
 *  matrix  - simple library for matrix operations
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "matrix.h"

float *buf = NULL;
unsigned int buflen = 0;


int matrix_bufAlloc(matrix_t *matrix, unsigned int rows, unsigned int cols)
{
	float *data;

	if (rows == 0 || cols == 0) {
		return -1;
	}

	/* check for calloc 'nitems' overflow */
	if (SIZE_MAX / rows < cols) {
		return -1;
	}

	data = calloc(rows * cols, sizeof(float));
	if (data == NULL) {
		return -1;
	}

	matrix->rows = rows;
	matrix->cols = cols;
	matrix->transposed = 0;
	matrix->data = data;

	return 0;
}

void matrix_bufFree(matrix_t *matrix)
{
	free(matrix->data);
	matrix->data = NULL;
}


void matrix_print(const matrix_t *A)
{
	unsigned int row, col;

	if (A->transposed) {
		for (row = 0; row < A->cols; row++) {
			for (col = 0; col < A->rows; col++) {
				printf("%f ", A->data[row + A->cols * col]);
			}
			printf("\n");
		}
	}
	else {
		for (row = 0; row < A->rows; row++) {
			for (col = 0; col < A->cols; col++) {
				printf("%f ", A->data[col + A->cols * row]);
			}
			printf("\n");
		}
	}
}


/* get element (row, col) from M that is transposed */
static inline float pos_trpd(const matrix_t *M, unsigned int row, unsigned int col)
{
	return M->data[M->cols * col + row];
}


/* get element (row, col) from M that is not transposed */
static inline float pos_norm(const matrix_t *M, unsigned int row, unsigned int col)
{
	return M->data[M->cols * row + col];
}


void matrix_trp(matrix_t *A)
{
	A->transposed = !A->transposed;
}


int matrix_prod(const matrix_t *A, const matrix_t *B, matrix_t *C)
{
	unsigned int row, col;                               /* represent position in output C matrix */
	unsigned int step;                                   /* represent stepping down over rows/columns in A and B */
	const unsigned int Acols = A->cols, Bcols = B->cols; /* rewritten Acols and Bcols for better performance */
	float currC;
	matrix_t tmp = { .data = C->data, .rows = matrix_rowsGet(C), .cols = matrix_colsGet(C), .transposed = 0 };

	if (A->transposed) {
		if (B->transposed) {
			/* both transposed logic */
			if (A->rows != B->cols || tmp.cols != B->rows || tmp.rows != A->cols) {
				return -1;
			}

			for (row = 0; row < tmp.rows; row++) {
				for (col = 0; col < tmp.cols; col++) {
					currC = 0;
					for (step = 0; step < A->rows; step++) {
						currC += A->data[row + Acols * step] * B->data[step + Bcols * col];
					}
					tmp.data[col + tmp.cols * row] = currC;
				}
			}
		}
		else {
			/* only A is transposed */
			if (A->rows != B->rows || tmp.cols != B->cols || tmp.rows != A->cols) {
				return -1;
			}

			for (row = 0; row < tmp.rows; row++) {
				for (col = 0; col < tmp.cols; col++) {
					currC = 0;
					for (step = 0; step < A->rows; step++) {
						currC += A->data[row + Acols * step] * B->data[col + Bcols * step];
					}
					tmp.data[col + tmp.cols * row] = currC;
				}
			}
		}
	}
	else {
		row = A->rows;
		if (B->transposed) {
			/* only B transposed logic */
			if (A->cols != B->cols || tmp.cols != B->rows || tmp.rows != A->rows) {
				return -1;
			}

			for (row = 0; row < tmp.rows; row++) {
				for (col = 0; col < tmp.cols; col++) {
					currC = 0;
					for (step = 0; step < Acols; step++) {
						currC += A->data[step + Acols * row] * B->data[step + Bcols * col];
					}
					tmp.data[col + tmp.cols * row] = currC;
				}
			}
		}
		else {
			/* no matrix is transposed */
			if (A->cols != B->rows || tmp.cols != B->cols || tmp.rows != A->rows) {
				return -1;
			}

			for (row = 0; row < tmp.rows; row++) {
				for (col = 0; col < tmp.cols; col++) {
					currC = 0;
					for (step = 0; step < A->cols; step++) {
						currC += A->data[step + Acols * row] * B->data[col + Bcols * step];
					}
					tmp.data[col + tmp.cols * row] = currC;
				}
			}
		}
	}

	C->cols = tmp.cols;
	C->rows = tmp.rows;
	C->transposed = tmp.transposed;

	return 0;
}


int matrix_sparseProd(const matrix_t *A, const matrix_t *B, matrix_t *C)
{
	unsigned int row, col; /* represent position in output A matrix */
	unsigned int step;     /* represent stepping down over rows/columns in A and B */
	float currA;
	matrix_t tmp = { .data = C->data, .rows = matrix_rowsGet(C), .cols = matrix_colsGet(C), .transposed = 0 };

	if (A->transposed) {
		if (B->transposed) {
			/* both matrix transposed */
			if (A->rows != B->cols || tmp.cols != B->rows || tmp.rows != A->cols) {
				return -1;
			}
			memset(tmp.data, 0, tmp.cols * tmp.rows * sizeof(float));

			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					currA = A->data[A->cols * col + row];
					if (currA != 0) {
						for (step = 0; step < tmp.cols; step++) {
							tmp.data[row * tmp.cols + step] += currA * B->data[B->cols * step + col];
						}
					}
				}
			}
		}
		else {
			/* only A is transposed */
			if (A->rows != B->rows || tmp.cols != B->cols || tmp.rows != A->cols) {
				return -1;
			}
			memset(tmp.data, 0, tmp.cols * tmp.rows * sizeof(float));

			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					currA = A->data[A->cols * col + row];
					if (currA != 0) {
						for (step = 0; step < tmp.cols; step++) {
							tmp.data[row * tmp.cols + step] += currA * B->data[B->cols * col + step];
						}
					}
				}
			}
		}
	}
	else {
		if (B->transposed) {
			/* only B is transposed */
			if (A->cols != B->cols || tmp.cols != B->rows || tmp.rows != A->rows) {
				return -1;
			}
			memset(tmp.data, 0, tmp.cols * tmp.rows * sizeof(float));

			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * row + col];

					if (currA != 0) {
						for (step = 0; step < tmp.cols; step++) {
							tmp.data[row * tmp.cols + step] += currA * B->data[B->cols * step + col];
						}
					}
				}
			}
		}
		else {
			/* no matrix is transposed */
			if (A->cols != B->rows || tmp.cols != B->cols || tmp.rows != A->rows) {
				return -1;
			}
			memset(tmp.data, 0, tmp.cols * tmp.rows * sizeof(float));

			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * row + col];

					if (currA != 0) {
						for (step = 0; step < tmp.cols; step++) {
							tmp.data[row * tmp.cols + step] += currA * B->data[B->cols * col + step];
						}
					}
				}
			}
		}
	}

	C->cols = tmp.cols;
	C->rows = tmp.rows;
	C->transposed = tmp.transposed;

	return 0;
}


static inline int matrix_sandwitchValid(const matrix_t *A, const matrix_t *B, const matrix_t *C, const matrix_t *tempC)
{
	unsigned int colsA = matrix_colsGet(A), rowsA = matrix_rowsGet(A);
	unsigned int colsB = matrix_colsGet(B), rowsB = matrix_rowsGet(B);
	unsigned int colsC = matrix_colsGet(C), rowsC = matrix_rowsGet(C);
	unsigned int colsTempC = matrix_colsGet(tempC), rowsTempC = matrix_rowsGet(tempC);

	return colsA == rowsB && rowsA == rowsTempC && colsB == colsTempC && /* First multiplication */
		colsTempC == colsA && rowsTempC == rowsC && rowsA == colsC;      /* Second multiplication */
}


int matrix_sparseSandwitch(const matrix_t *A, const matrix_t *B, matrix_t *C, matrix_t *tempC)
{
	const matrix_t trpA = { .data = A->data, .rows = A->rows, .cols = A->cols, .transposed = 1 };

	if (!matrix_sandwitchValid(A, B, C, tempC)) {
		return -1;
	}

	matrix_sparseProd(A, B, tempC);
	matrix_sparseProd(tempC, &trpA, C);
	return 0;
}


int matrix_sandwitch(const matrix_t *A, const matrix_t *B, matrix_t *C, matrix_t *tempC)
{
	const matrix_t trpA = { .data = A->data, .rows = A->rows, .cols = A->cols, .transposed = 1 };

	if (!matrix_sandwitchValid(A, B, C, tempC)) {
		return -1;
	}

	matrix_prod(A, B, tempC);
	matrix_prod(tempC, &trpA, C);
	return 0;
}


void matrix_diag(matrix_t *A)
{
	int i;

	matrix_zeroes(A);
	for (i = 0; i < A->cols && i < A->rows; i++) {
		A->data[A->cols * i + i] = 1.F;
	}
}


/* performs A + B = C, or A +=B if C is NULL */
int matrix_add(matrix_t *A, const matrix_t *B, matrix_t *C)
{
	unsigned int row, col; /* represent position in output C matrix */
	unsigned int rowsC = A->rows, colsC = A->cols;

	if (C != NULL) {
		if (!(C->rows == A->rows && C->cols == A->cols) && !(C->rows == A->cols && C->cols == A->rows)) {
			return -1;
		}
	}
	else {
		C = A;
	}

	/* different B matrix indexing only if one of B or A is transposed */
	if (!A->transposed == !B->transposed) {
		if (A->rows != B->rows || A->cols != B->cols) {
			return -1;
		}

		for (row = 0; row < rowsC; row++) {
			for (col = 0; col < colsC; col++) {
				C->data[colsC * row + col] = A->data[A->cols * row + col] + B->data[B->cols * row + col];
			}
		}
	}
	else {
		if (A->rows != B->cols || A->cols != B->rows) {
			return -1;
		}

		for (row = 0; row < rowsC; row++) {
			for (col = 0; col < colsC; col++) {
				C->data[colsC * row + col] = A->data[A->cols * row + col] + B->data[B->cols * col + row];
			}
		}
	}

	if (C != A) {
		C->rows = rowsC;
		C->cols = colsC;
		C->transposed = A->transposed;
	}

	return 0;
}

/* performs A + B = C, or A -=B if C is NULL */
int matrix_sub(matrix_t *A, const matrix_t *B, matrix_t *C)
{
	unsigned int row, col; /* represent position in output C matrix */
	unsigned int rowsC = A->rows, colsC = A->cols;

	if (C != NULL) {
		if (!(C->rows == A->rows && C->cols == A->cols) && !(C->rows == A->cols && C->cols == A->rows)) {
			return -1;
		}
	}
	else {
		C = A;
	}

	/* different B matrix indexing only if one of B or A is transposed */
	if (!A->transposed == !B->transposed) {
		if (A->rows != B->rows || A->cols != B->cols) {
			return -1;
		}

		for (row = 0; row < rowsC; row++) {
			for (col = 0; col < colsC; col++) {
				C->data[colsC * row + col] = A->data[A->cols * row + col] - B->data[B->cols * row + col];
			}
		}
	}
	else {
		if (A->rows != B->cols || A->cols != B->rows) {
			return -1;
		}

		for (row = 0; row < rowsC; row++) {
			for (col = 0; col < colsC; col++) {
				C->data[colsC * row + col] = A->data[A->cols * row + col] - B->data[B->cols * col + row];
			}
		}
	}

	if (C != A) {
		C->rows = rowsC;
		C->cols = colsC;
		C->transposed = A->transposed;
	}

	return 0;
}


int matrix_cmp(const matrix_t *A, const matrix_t *B)
{
	unsigned int row, col; /* represent position in output C matrix */

	if (A->transposed) {
		if (B->transposed) {
			if (A->cols != B->cols || A->rows != B->rows) {
				return -1;
			}

			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					if (pos_trpd(A, row, col) != pos_trpd(B, row, col)) {
						return -1;
					}
				}
			}
		}
		else {
			if (A->cols != B->rows || A->rows != B->cols) {
				return -1;
			}

			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					if (pos_trpd(A, row, col) != pos_norm(B, row, col)) {
						return -1;
					}
				}
			}
		}
	}
	else {
		if (B->transposed) {
			if (A->cols != B->rows || A->rows != B->cols) {
				return -1;
			}


			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					if (pos_norm(A, row, col) != pos_trpd(B, row, col)) {
						return -1;
					}
				}
			}
		}
		else {
			if (A->cols != B->cols || A->rows != B->rows) {
				return -1;
			}

			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					if (pos_norm(A, row, col) != pos_norm(B, row, col)) {
						return -1;
					}
				}
			}
		}
	}
	return 0;
}


int matrix_inv(const matrix_t *A, matrix_t *B, float *buf, int buflen)
{
	matrix_t C = { 0 };
	int rows, cols, row, col, step;
	float base;

	if (A->rows != A->cols || B->rows != B->cols || A->rows != B->rows) {
		return -1;
	}

	if (buflen < A->rows * A->cols * 2) {
		return -1;
	}

	/* create working matrix */
	if (A->transposed) {
		rows = A->cols;
		cols = A->rows;
		C = (matrix_t) { .rows = cols, .cols = rows * 2, .data = buf };
	}
	else {
		rows = A->rows;
		cols = A->cols;
		C = (matrix_t) { .rows = rows, .cols = cols * 2, .data = buf };
	}
	memset(C.data, 0, sizeof(float) * buflen);

	/* write to working matrix A and identity */
	for (row = 0; row < rows; row++) {
		C.data[C.cols * row + cols + row] = 1;
		for (col = 0; col < cols; col++) {
			C.data[C.cols * row + col] = (A->transposed) ? pos_trpd(A, row, col) : pos_norm(A, row, col);
		}
	}

	/* lower triangle elimination */
	for (col = 0; col < cols; col++) {
		base = C.data[C.cols * col + col];
		if (base == 0) {
			return -1;
		}

		for (step = 0; step < C.cols; step++) {
			C.data[C.cols * col + step] /= base;
		}
		/* now left submatrix has ones on diagonal element of row[col] */

		for (row = col + 1; row < C.rows; row++) {
			base = C.data[C.cols * row + col];
			for (step = 0; step < C.cols; step++) {
				C.data[C.cols * row + step] -= base * C.data[C.cols * col + step];
			}
		}
	}

	/* upper triangle elimination +*/
	for (col = cols - 1; col >= 0; col--) {
		base = C.data[C.cols * col + col];

		for (row = col - 1; row >= 0; row--) {
			base = C.data[C.cols * row + col];
			for (step = col; step < C.cols; step++) {
				C.data[C.cols * row + step] -= base * C.data[C.cols * col + step];
			}
		}
	}

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			B->data[B->cols * row + col] = C.data[C.cols * row + col + cols];
		}
	}
	B->transposed = 0;
	return 0;
}

int matrix_writeSubmatrix(matrix_t *dst, unsigned int row, unsigned int col, const matrix_t *src)
{
	int cprow;

	if (col + src->cols > dst->cols || row + src->rows > dst->rows) {
		return -1;
	}

	for (cprow = 0; cprow < src->rows; cprow++) {
		memcpy((char *)&dst->data[dst->cols * (cprow + row) + col], (char *)&src->data[src->cols * cprow], sizeof(float) * src->cols);
	}

	return 0;
}


void matrix_zeroes(matrix_t *A)
{
	memset(A->data, 0, sizeof(float) * A->rows * A->cols);
}


void matrix_times(matrix_t *A, float scalar)
{
	int i;
	for (i = 0; i < A->rows * A->cols; i++) {
		A->data[i] *= scalar;
	}
}
