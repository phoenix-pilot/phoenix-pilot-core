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

#define DEG2RAD 0.0174532925


typedef struct {
	unsigned int rows;
	unsigned int cols;
	unsigned int transposed;
	float *data;
} phmatrix_t;

/* sets all matrix to zeroes */
extern void phx_zeroes(phmatrix_t *A);


extern int phx_newmatrix(phmatrix_t *matrix, int rows, int cols);


extern void phx_matrixDestroy(phmatrix_t *matrix);


/* sets the matrix data to diagonal 1s, rest is 0 */
extern void phx_diag(phmatrix_t *A);


/* multiplies matrix terms by scalar */
extern void phx_scalar_product(phmatrix_t *A, float scalar);


/* prints matrix A to standard output */
extern void phx_print(phmatrix_t *A);


/* transposes matrix A */
extern void phx_transpose(phmatrix_t *A);


/* overwrites C with A * B */
extern int phx_product(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C);


/* overwrites C with A * B, optimized for sparse A matrix */
extern int phx_product_sparse(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C);


/* overwrites C with A * B * transposed(A), optimized for sparse A */
extern int phx_sadwitch_product(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C, phmatrix_t *tempC);


/* phx_sadwitch_product() optimized for sparse A matrix */
extern int phx_sadwitch_product_sparse(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C, phmatrix_t *tempC);


/* if C is not null perform C = A + B, otherwise A += B */
extern int phx_add(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C);


/* if C is not null perform C = A - B, otherwise A += B */
extern int phx_sub(phmatrix_t *A, phmatrix_t *B, phmatrix_t *C);


/* compares contents of A and B */
extern int pxh_compare(phmatrix_t *A, phmatrix_t *B);


/* calculates inverse matrix */
extern int phx_inverse(phmatrix_t *A, phmatrix_t *B, float *buf, int buflen);


/* writes submatrix B into matrix A beginning from position A(row, col). Works only for non-transposed matrices */
extern void phx_writesubmatrix(phmatrix_t *A, int row, int col, phmatrix_t *B);


#endif /* PHMATRIX_H */
