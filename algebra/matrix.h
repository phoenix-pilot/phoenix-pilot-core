/*
 * Phoenix-Pilot
 *
 * matrix - simple library for matrix operations - header file
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

typedef struct {
	unsigned int rows;
	unsigned int cols;
	unsigned int transposed;
	float *data;
} matrix_t;

/* sets all matrix to zeroes */
extern void matrix_zeroes(matrix_t *A);


extern int matrix_alloc(matrix_t *matrix, int rows, int cols);


extern void matrix_dealloc(matrix_t *matrix);


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


/* if C is not null perform C = A - B, otherwise A += B */
extern int matrix_sub(matrix_t *A, matrix_t *B, matrix_t *C);


/* compares contents of A and B */
extern int matrix_cmp(matrix_t *A, matrix_t *B);


/* calculates inverse matrix */
extern int matrix_inv(matrix_t *A, matrix_t *B, float *buf, int buflen);


/* writes submatrix B into matrix A beginning from position A(row, col). Works only for non-transposed matrices */
extern void matrix_writeSubmatrix(matrix_t *A, int row, int col, matrix_t *B);


#endif /* PHMATRIX_H */
