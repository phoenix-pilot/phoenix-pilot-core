/*
 * Phoenix-Pilot
 *
 * matrix - simple library for matrix operations
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PHMATRIX_H
#define PHMATRIX_H

#include <stddef.h>

typedef struct {
	unsigned int rows;
	unsigned int cols;
	unsigned int transposed;
	float *data;
} matrix_t;

/* sets all matrix to zeroes */
extern void matrix_zeroes(matrix_t *A);


extern int matrix_bufAlloc(matrix_t *matrix, unsigned int rows, unsigned int cols);


extern void matrix_bufFree(matrix_t *matrix);


/* return a pointer to matrix element in specified row and column
 * if row or column exceeds matrix size NULL is returned */
static inline float *matrix_at(const matrix_t *A, unsigned int row, unsigned int col)
{
	return (A->transposed) ? ((row < A->cols && col < A->rows) ? &(A->data[A->cols * col + row]) : NULL) : ((row < A->rows && col < A->cols) ? &(A->data[A->cols * row + col]) : NULL);
}


static inline unsigned int matrix_rowsGet(matrix_t *A)
{
	return (A->transposed) ? A->cols : A->rows;
}


static inline unsigned int matrix_colsGet(matrix_t *A)
{
	return (A->transposed) ? A->rows : A->cols;
}


/* sets the matrix data to diagonal 1s, rest is 0 */
extern void matrix_diag(matrix_t *A);


/* multiplies matrix terms by scalar */
extern void matrix_times(matrix_t *A, float scalar);


/* prints matrix A to standard output */
extern void matrix_print(matrix_t *A);


/* transposes matrix A */
extern void matrix_trp(matrix_t *A);


/* overwrites C with A * B */
extern int matrix_prod(matrix_t *A, matrix_t *B, matrix_t *C);


/* overwrites C with A * B, optimized for sparse A matrix */
extern int matrix_sparseProd(matrix_t *A, matrix_t *B, matrix_t *C);


/* overwrites C with A * B * transposed(A), optimized for sparse A */
extern int matrix_sandwitch(matrix_t *A, matrix_t *B, matrix_t *C, matrix_t *tempC);


/* matrix_sandwitch() optimized for sparse A matrix */
extern int matrix_sparseSandwitch(matrix_t *A, matrix_t *B, matrix_t *C, matrix_t *tempC);


/* if C is not null perform C = A + B, otherwise A += B */
extern int matrix_add(matrix_t *A, matrix_t *B, matrix_t *C);


/* if C is not null perform C = A - B, otherwise A -= B */
extern int matrix_sub(matrix_t *A, matrix_t *B, matrix_t *C);


/* compares contents of A and B */
extern int matrix_cmp(const matrix_t *A, const matrix_t *B);


/* calculates inverse matrix, `buf` have to be twice as big as matrix `A`, bufLen >= A->rows * A->cols * 2 */
extern int matrix_inv(matrix_t *A, matrix_t *B, float *buf, int buflen);


/* writes submatrix `src` into matrix `dst` beginning from position dst(row, col). Works only for non-transposed matrices */
extern int matrix_writeSubmatrix(matrix_t *dst, unsigned int row, unsigned int col, matrix_t *src);


#endif /* PHMATRIX_H */
