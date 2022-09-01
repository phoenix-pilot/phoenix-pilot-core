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

#ifndef MATRIX_TEST_TOOLS_H
#define MATRIX_TEST_TOOLS_H

#include <matrix.h>

/* Defines for matrix_bufAlloc results */
#define BUF_ALLOC_OK   0
#define BUF_ALLOC_FAIL -1

/* Defines for functions that validate matrix parameters */
#define CHECK_OK   0
#define CHECK_FAIL -1

#define PRODUCT_OK   0
#define PRODUCT_FAIL -1

#define WRITE_SUBMAT_OK   0
#define WRITE_SUBMAT_FAIL -1

#define MAT_CMP_OK 0

/* Defines used by tests */

/* Must be at least 1 */
#define SMALL_SHIFT 1

/* Must be bigger than SMALL_SHIFT */
#define BIG_SHIFT 1234


/* ##############################################################################
 * ---------------------        matrix modification       -----------------------
 * ############################################################################## */


/* Fill matrix buffer with n-length array vals. If n=1 whole matrix is filled with vals[0] */
extern void algebraTests_buffFill(matrix_t *M, const float *vals, unsigned int n);


/* Create and fill matrix with n-length array vals. If n=1 whole matrix is filled with vals[0]. M must be uninitialized */
extern int algebraTests_createAndFill(matrix_t *M, unsigned int rows, unsigned int cols, const float *vals, unsigned int n);


/* Copies src to des. Destination matrix have to be uninitiated */
extern int algebraTests_matrixCopy(matrix_t *des, matrix_t *src);


/* Transposes matrix by rearranging its elements. Does not change M->transposed */
extern int algebraTests_realTrp(matrix_t *M);


/* Transpose `M` using matrix_trp() and swap its memory so it is also transposed. Does not change `M` in mathematical meaning */
extern int algebraTests_transposeSwap(matrix_t *M);


/* ##############################################################################
 * ------------------------        matrix checks       --------------------------
 * ############################################################################## */


/* Function checks if every matrix element is zero */
extern int algebraTests_matrixZeroesCheck(matrix_t *A);


/* Function checks if matrix_at return a NULL, when we are trying to get matrix element outside matrix */
extern int algebraTests_invalidSeekCheck(matrix_t *M);


/* Checks if M2 corresponds to M1 physically transposed (with swapped data in buffer, and switched cols and rows) */
extern int algebraTests_diagCheck(matrix_t *M);


/* Checks if M2 is equal to M1 transposed */
extern int algebraTests_dataTrpCheck(matrix_t *M1, matrix_t *M2);


/* Checks if M1 is identical to M2. Different between matrix_cmp is that .transposed flag must be equal in both matrices */
extern int algebraTest_equalMatrix(const matrix_t *M1, const matrix_t *M2);


/* Checks if `M` is equal to `dst` with `src` printed in `col` and `row`. All matrices have to be non-transposed*/
extern int algebraTests_submatCheck(const matrix_t *dst, unsigned int row, unsigned int col, const matrix_t *src, const matrix_t *M);

#endif
