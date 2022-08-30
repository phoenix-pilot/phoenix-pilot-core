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

#include <stdlib.h>
#include <string.h>


#define SMALL_SHIFT 1
#define BIG_SHIFT   1234


void algebraTests_buffFill(matrix_t *M, const float *vals, unsigned int n)
{
	int rowsNum, colsNum, row, col;
	int i = 0;

	rowsNum = matrix_rowsGet(M);
	colsNum = matrix_colsGet(M);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			*matrix_at(M, row, col) = (n > 1) ? vals[i++] : vals[i];
		}
	}
}


int algebraTests_createAndFill(matrix_t *M, unsigned int rows, unsigned int cols, const float *vals, unsigned int n)
{
	if (matrix_bufAlloc(M, rows, cols) != BUF_ALLOC_OK) {
		return BUF_ALLOC_FAIL;
	}

	algebraTests_buffFill(M, vals, n);
	return BUF_ALLOC_OK;
}


int algebraTests_matrixCopy(matrix_t *des, matrix_t *src)
{
	if (algebraTests_createAndFill(des, src->rows, src->cols, src->data, src->rows * src->cols) != BUF_ALLOC_OK) {
		return BUF_ALLOC_FAIL;
	}

	des->transposed = src->transposed;
	return BUF_ALLOC_OK;
}


int algebraTests_realTrp(matrix_t *M)
{
	/* As its turns out it is not easy to transpose matrix without additional buffer */
	/* https://en.wikipedia.org/wiki/In-place_matrix_transposition */
	/* This solution is fast and easier to understand */

	int rowsNum, colsNum, row, col;
	matrix_t tmpM;

	if (matrix_bufAlloc(&tmpM, M->cols, M->rows) != BUF_ALLOC_OK) {
		return BUF_ALLOC_FAIL;
	}
	tmpM.transposed = M->transposed;

	rowsNum = matrix_rowsGet(M);
	colsNum = matrix_colsGet(M);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			*matrix_at(&tmpM, col, row) = *matrix_at(M, row, col);
		}
	}

	memcpy(M->data, tmpM.data, sizeof(float) * M->rows * M->cols);
	matrix_bufFree(&tmpM);

	M->cols = tmpM.cols;
	M->rows = tmpM.rows;

	return BUF_ALLOC_OK;
}


int algebraTests_invalidSeekCheck(matrix_t *M)
{
	int rowsNum, colsNum, row, col;

	rowsNum = matrix_rowsGet(M);
	colsNum = matrix_colsGet(M);

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


int algebraTests_matrixZeroesCheck(matrix_t *A)
{
	int rowsNum, colsNum, row, col;

	rowsNum = matrix_rowsGet(A);
	colsNum = matrix_colsGet(A);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			if (*matrix_at(A, row, col) != 0.0) {
				return CHECK_FAIL;
			}
		}
	}

	return CHECK_OK;
}


int algebraTests_diagCheck(matrix_t *M)
{
	int rowsNum, colsNum, row, col;

	rowsNum = matrix_rowsGet(M);
	colsNum = matrix_colsGet(M);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			if (row == col && *matrix_at(M, row, col) != 1.0) {
				return CHECK_FAIL;
			}
			if (row != col && *matrix_at(M, row, col) != 0.0) {
				return CHECK_FAIL;
			}
		}
	}

	return CHECK_OK;
}
