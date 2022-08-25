/*
 * Phoenix-Pilot
 *
 * Tools for matrix library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <matrix.h>

/* Defines for matrix_bufAlloc results */
#define BUF_ALLOC_OK   0
#define BUF_ALLOC_FAIL -1

/* Defines for functions that validate matrix parameters */
#define CHECK_OK   0
#define CHECK_FAIL -1


/* ##############################################################################
 * ---------------------        matrix modification       -----------------------
 * ############################################################################## */


/* Fill matrix buffer with n-length array vals. If n=1 whole matrix is filled with vals[0] */
extern void algebraTests_buffFill(matrix_t *M, const float *vals, unsigned int n);


/* Create and fill matrix with n-length array vals. If n=1 whole matrix is filled with vals[0]. M must be uninitialized */
extern int algebraTests_createAndFill(matrix_t *M, unsigned int rows, unsigned int cols, const float *vals, unsigned int n);


/* Copies src to des. Destination matrix have to be uninitiated */
extern int algebraTests_matrixCopy(matrix_t *src, matrix_t *des);


/* ##############################################################################
 * ------------------------        matrix checks       --------------------------
 * ############################################################################## */


/* Function checks if every matrix element is zero */
extern int algebraTests_matrixZeroesCheck(matrix_t *A);


/* Function checks if matrix_at return a NULL, when we are trying to get matrix element outside matrix */
extern int algebraTests_invalidSeekCheck(matrix_t *M);


/* This function checks if all elements on diagonal are ones and others are zeroes */
extern int algebraTests_diagCheck(matrix_t *M);
