/*
 * Phoenix-Pilot
 *
 * phmatrix
 *
 * simple library for matrix operations
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
#include "phmatrix.h"

float * buf = NULL;
unsigned int buflen = 0;


int phx_newmatrix(phmatrix_t *matrix, int rows, int cols)
{
	float *data;

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

void phx_matrixDestroy(phmatrix_t *matrix)
{
	if (matrix->data != NULL) {
		free(matrix->data);
		matrix->data = NULL;
	}
}


void phx_print(phmatrix_t * A)
{
	unsigned int row, col;

	if (A->transposed) {
		for (row = 0; row < A->cols; row++) {
			for (col = 0; col < A->rows; col++ ) {
				printf("%f ", A->data[row + A->cols * col]);
			}
			printf("\n");
		}
	}
	else {
		for (row = 0; row < A->rows; row++) {
			for (col = 0; col < A->cols; col++ ) {
				printf("%f ", A->data[col + A->cols * row]);
			}
			printf("\n");
		}
	}
}


/* get element (row, col) from M that is transposed */
static inline float pos_trpd(phmatrix_t * M, unsigned int row, unsigned int col) {
	return M->data[M->cols * col + row];
}


/* get element (row, col) from M that is not transposed */
static inline float pos_norm(phmatrix_t * M, unsigned int row, unsigned int col) {
	return M->data[M->cols * row + col];
}


void phx_transpose(phmatrix_t *A)
{
	A->transposed = !A->transposed;
}


int phx_product(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C)
{
	unsigned int row, col; /* represent position in output C matrix */
	unsigned int step; /* represent stepping down over rows/columns in A and B */
	const unsigned int Acols = A->cols, Bcols = B->cols; /* rewritten Acols and Bcols for better performance */
	float currC;

	memset(C->data, 0, sizeof(float) * C->rows * C->cols);
	/* force switch C matrix to untransposed */
	if (C->transposed) {
		row = C->rows;
		C->rows = C->cols;
		C->cols = row;
		C->transposed = 0;
	}

	if (A->transposed) {
		if (B->transposed) {
			/* both transposed logic */
			for (row = 0; row < C->rows; row++) {
				for (col = 0; col < C->cols; col++) {
					currC = 0;
					for (step = 0; step < A->rows; step++) {
						currC += A->data[row + Acols * step] * B->data[step + Bcols * col];
					}
					C->data[col + C->cols * row] = currC;
				}
			}
		}
		else {
			/* only A is transposed */
			for (row = 0; row < C->rows; row++) {
				for (col = 0; col < C->cols; col++) {
					currC = 0;
					for (step = 0; step < A->rows; step++) {
						currC += A->data[row + Acols * step] * B->data[col + Bcols * step];
					}
					C->data[col + C->cols * row] = currC;
				}
			}
		}
	}
	else {
		row = A->rows;
		if (B->transposed) {
			/* only B transposed logic */
			for (row = 0; row < C->rows; row++) {
				for (col = 0; col < C->cols; col++) {
					currC = 0;
					for (step = 0; step < Acols; step++) {
						currC += A->data[step + Acols * row] * B->data[step + Bcols * col];
					}
					C->data[col + C->cols * row] = currC;
				}
			}
		}
		else {
			/* no matrix is transposed */
			for (row = 0; row < C->rows; row++) {
				for (col = 0; col < C->cols; col++) {
					currC = 0;
					for (step = 0; step < A->cols; step++) {
						currC += A->data[step + Acols * row] * B->data[col + Bcols * step];
					}
					C->data[col + C->cols * row] = currC;
				}
			}
		}
	}
	return 0;
}


int phx_product_sparse(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C)
{
	unsigned int row, col; /* represent position in output A matrix */
	unsigned int step; /* represent stepping down over rows/columns in A and B */
	//const unsigned int Acols = A->cols, Bcols = B->cols; /* rewritten Acols and Bcols for better performance */
	float currA;

	memset(C->data, 0, sizeof(float) * C->rows * C->cols);
	/* force switch C matrix to untransposed */
	if (C->transposed) {
		row = C->rows;
		C->rows = C->cols;
		C->cols = row;
		C->transposed = 0;
	}

	if (A->transposed) {
		if (B->transposed) {
			/* both matrix transposed */
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * col + row];
					if (currA == 0)
						continue;
					for (step = 0; step < C->cols; step++) {
						C->data[row * C->cols + step] += currA * B->data[B->cols * step + col];
					}
				}
			}
		}
		else {
			/* only A is transposed */
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * col + row];
					if (currA == 0)
						continue;
					for (step = 0; step < C->cols; step++) {
						C->data[row * C->cols + step] += currA * B->data[B->cols * col + step];
					}
				}
			}
		}
	}
	else {
		if (B->transposed) {
			/* only B is transposed */
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * row + col];
					if (currA == 0)
						continue;
					for (step = 0; step < C->cols; step++) {
						C->data[row * C->cols + step] += currA * B->data[B->cols * step + col];
					}
				}
			}
		}
		else {
			/* no matrix is transposed */
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					currA = A->data[A->cols * row + col];
					if (currA == 0)
						continue;
					for (step = 0; step < C->cols; step++) {
						C->data[row * C->cols + step] += currA * B->data[B->cols * col + step];
					}
				}
			}
		}
	}
	return 0;
}


int phx_sadwitch_product_sparse(phmatrix_t * A, phmatrix_t * B, phmatrix_t * C, phmatrix_t * tempC)
{
	phx_product_sparse(A, B, tempC);
	phx_transpose(A);
	phx_product_sparse(A, tempC, C);
	phx_transpose(A);
	return 0;
}


int phx_sadwitch_product(phmatrix_t * A, phmatrix_t * B, phmatrix_t * C, phmatrix_t * tempC)
{
	phx_product(A, B, tempC);
	phx_transpose(A);
	phx_product(tempC, A, C);
	phx_transpose(A);
	return 0;
}


void phx_diag(phmatrix_t * A)
{
	int i;

	phx_zeroes(A);
	for(i = 0; i < A->cols && i < A->rows; i++) {
		A->data[A->cols * i + i] = 1.F;
	}
}


/* performs A + B = C, or A +=B if C is NULL */
int phx_add(phmatrix_t * A, phmatrix_t * B, phmatrix_t * C)
{
	unsigned int row, col; /* represent position in output C matrix */

	if (C != NULL) {
		/* force switch C matrix to untransposed */
		memset(C->data, 0, sizeof(float) * C->rows * C->cols);
		/* force switch C matrix to untransposed */
		if (C->transposed) {
			row = C->rows;
			C->rows = C->cols;
			C->cols = row;
			C->transposed = A->transposed;
		}
	}
	else {
		C = A;
	}

	/* different B matrix indexing only one of B or A is transposed */
	if (!A->transposed == !B->transposed){
		for (row = 0; row < C->rows; row++) {
			for(col = 0; col < C->cols; col++) {
				C->data[C->cols * row + col] = A->data[A->cols * row + col] + B->data[B->cols * row + col];
			}
		}
	}
	else {
		for (row = 0; row < C->rows; row++) {
			for(col = 0; col < C->cols; col++) {
				C->data[C->cols * row + col] = A->data[A->cols * row + col] + B->data[B->cols * col + row];
			}
		}
	}
	return 0;
}

/* performs A + B = C, or A -=B if C is NULL */
int phx_sub(phmatrix_t * A, phmatrix_t * B, phmatrix_t * C)
{
	unsigned int row, col; /* represent position in output C matrix */

	if (C != NULL) {
		memset(C->data, 0, sizeof(float) * C->rows * C->cols);
		/* force switch C matrix to untransposed */
		if (C->transposed) {
			row = C->rows;
			C->rows = C->cols;
			C->cols = row;
			C->transposed = A->transposed;
		}
	}
	else {
		/* if C is NULL addition happen into A so treat A as C */
		C = A;
	}

	/* different B matrix indexing only one of B or A is transposed */
	if (!A->transposed == !B->transposed){
		for (row = 0; row < C->rows; row++) {
			for(col = 0; col < C->cols; col++) {
				C->data[C->cols * row + col] = A->data[A->cols * row + col] - B->data[B->cols * row + col];
			}
		}
	}
	else {
		for (row = 0; row < C->rows; row++) {
			for(col = 0; col < C->cols; col++) {
				C->data[C->cols * row + col] = A->data[A->cols * row + col] - B->data[B->cols * col + row];
			}
		}
	}
	return 0;
}


int pxh_compare(phmatrix_t * A, phmatrix_t * B)
{
	unsigned int row, col; /* represent position in output C matrix */

	if (A->transposed) {
		if (B->transposed) {
			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					if (pos_trpd(A, row, col) != pos_trpd(B, row, col))
						return -1;
				}
			}
		}
		else {
			for (row = 0; row < A->cols; row++) {
				for (col = 0; col < A->rows; col++) {
					if (pos_trpd(A, row, col) != pos_norm(B, row, col))
						return -1;
				}
			}
		}
	}
	else {
		if (B->transposed) {
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					if (pos_norm(A, row, col) != pos_trpd(B, row, col))
						return -1;
				}
			}
		}
		else {
			for (row = 0; row < A->rows; row++) {
				for (col = 0; col < A->cols; col++) {
					if (pos_norm(A, row, col) != pos_norm(B, row, col))
						return -1;
				}
			}
		}
	}
	return 0;
}


int phx_inverse(phmatrix_t * A, phmatrix_t * B, float * buf, int buflen)
{
	phmatrix_t C = {0};
	int rows, cols, row, col, step;
	float base;

	/* create working matrix */
	if (A->transposed) {
		rows = A->cols;
		cols = A->rows;
		C = (phmatrix_t) { .rows = cols, .cols = rows * 2, .data = buf };
	}
	else {
		rows = A->rows;
		cols = A->cols;
		C = (phmatrix_t) { .rows = rows, .cols = cols * 2, .data = buf };
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
		for (step = 0; step < C.cols; step++) {
			C.data[C.cols * col + step] /= base;
		}
		/* now left submatrix has ones on diagonal element of row[col] */

		for (row = col+1; row < C.rows; row++) {
			base = C.data[C.cols * row + col];
			for (step = 0; step < C.cols; step++) {
				C.data[C.cols * row + step] -= base * C.data[C.cols * col + step];
			}
		}
	}

	/* upper triangle elimination +*/
	for (col = cols-1; col >= 0; col--) {
		base = C.data[C.cols * col + col];

		for (row = col-1; row >= 0; row--) {
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

void phx_writesubmatrix(phmatrix_t *A, int row, int col, phmatrix_t *B){
	int cprow;

	for (cprow = 0; cprow < B->rows ; cprow++)
	{
		memcpy((char*)&A->data[A->cols * (cprow + row) + col], (char*)&B->data[B->cols * cprow], sizeof(float) * B->cols);
	}
}


void phx_zeroes(phmatrix_t *A)
{
	memset(A->data, 0, sizeof(float) * A->rows * A->cols);
}


void phx_scalar_product(phmatrix_t *A, float scalar)
{
	int i;
	for (i = 0; i < A->rows * A->cols; i++) {
		A->data[i] *= scalar;
	}
}
