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


/* Change every matrix element to val */
/* Function does not change sizes and transposition of matrix */
extern void algebraTests_valFill(matrix_t *M, float val);


/* Function checks if every matrix element is zero */
extern int algebraTests_matrixZeroesCheck(matrix_t *A);


/* Function checks if matrix_at return a NULL, when we are trying to get matrix element outside matrix */
extern int algebraTests_checkInvalidSeek(matrix_t *M);
