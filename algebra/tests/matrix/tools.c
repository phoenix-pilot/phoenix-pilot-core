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

#include "tools.h"


#define SMALL_SHIFT 1
#define BIG_SHIFT   1234


void algebraTests_getRowColNum(matrix_t *M, int *rows, int *cols)
{
	if (M->transposed) {
		*rows = M->cols;
		*cols = M->rows;
	}
	else {
		*rows = M->rows;
		*cols = M->cols;
	}
}


void algebraTests_fillWithVal(matrix_t *M, float val)
{
	int rowsNum, colsNum, row, col;

	algebraTests_getRowColNum(M, &rowsNum, &colsNum);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			*matrix_at(M, row, col) = val;
		}
	}
}


int algebraTests_checkInvalidSeek(matrix_t *M)
{
	int rowsNum, colsNum, row, col;

	algebraTests_getRowColNum(M, &rowsNum, &colsNum);

	/* Both row and col outside matrix */
	if (matrix_at(M, rowsNum, colsNum) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, rowsNum + SMALL_SHIFT, colsNum + SMALL_SHIFT) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, rowsNum + BIG_SHIFT, colsNum + BIG_SHIFT) != NULL) {
		return CHECK_FAIL;
	}

	/* Only row outside matrix */
	col = colsNum / 2; /* arbitrary position within columns */

	if (matrix_at(M, rowsNum, col) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, rowsNum + SMALL_SHIFT, col) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, rowsNum + BIG_SHIFT, col) != NULL) {
		return CHECK_FAIL;
	}

	/* Only col outside matrix */
	row = rowsNum / 2; /* arbitrary position within rows */

	if (matrix_at(M, row, colsNum) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, row, colsNum + SMALL_SHIFT) != NULL) {
		return CHECK_FAIL;
	}
	if (matrix_at(M, row, colsNum + BIG_SHIFT) != NULL) {
		return CHECK_FAIL;
	}

	return CHECK_OK;
}


int algebraTests_checkMatrixZeroes(matrix_t *A)
{
	int rowsNum, colsNum, row, col;

	algebraTests_getRowColNum(A, &rowsNum, &colsNum);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			if (*matrix_at(A, row, col) != 0.0) {
				return CHECK_FAIL;
			}
		}
	}

	return CHECK_OK;
}
