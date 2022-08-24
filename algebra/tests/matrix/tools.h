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


/* Function rewrites rows and cols with number of rows and columns of passed matrix. */
/* Function takes into considerations matrix transposition */
extern void algebraTests_getRowColNum(matrix_t *M, int *rows, int *cols);

/* Change every matrix element to val */
/* Function does not change sizes and transposition of matrix */
extern void algebraTests_fillWithVal(matrix_t *M, float val);
