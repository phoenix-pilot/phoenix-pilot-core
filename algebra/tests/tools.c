/*
 * Phoenix-Pilot
 *
 * Tools for algebra library unit tests
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


void test_assert_float_array_within(float delta, float *expected, float *actual, unsigned int elemNum, int line, char *message)
{
	unsigned int i;

	for (i = 0; i < elemNum; i++) {
		UNITY_TEST_ASSERT_FLOAT_WITHIN(delta, expected[i], actual[i], line, message);
	}
}


int algebraTests_buffFill(matrix_t *M, const float *vals, int n)
{
	int rowsNum, colsNum, row, col;
	int i = 0;

	rowsNum = matrix_rowsGet(M);
	colsNum = matrix_colsGet(M);

	if (n != rowsNum * colsNum && n != BUFFILL_WRITE_ALL) {
		return -1;
	}

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			*matrix_at(M, row, col) = (n == BUFFILL_WRITE_ALL) ? vals[i] : vals[i++];
		}
	}

	return 0;
}


int algebraTests_createAndFill(matrix_t *M, unsigned int rows, unsigned int cols, const float *vals, unsigned int n)
{
	if (matrix_bufAlloc(M, rows, cols) != MAT_BUF_ALLOC_OK) {
		return MAT_BUF_ALLOC_FAIL;
	}

	return algebraTests_buffFill(M, vals, n);
}


int algebraTests_matrixCopy(matrix_t *des, matrix_t *src)
{
	if (algebraTests_createAndFill(des, src->rows, src->cols, src->data, src->rows * src->cols) != MAT_BUF_ALLOC_OK) {
		return MAT_BUF_ALLOC_FAIL;
	}

	des->transposed = src->transposed;
	return MAT_BUF_ALLOC_OK;
}

int algebraTests_realTrp(matrix_t *M)
{
	/* As its turns out it is not easy to transpose matrix without additional buffer */
	/* https://en.wikipedia.org/wiki/In-place_matrix_transposition */
	/* This solution is fast and easier to understand */

	int rowsNum, colsNum, row, col;
	matrix_t tmpM;

	if (matrix_bufAlloc(&tmpM, M->cols, M->rows) != MAT_BUF_ALLOC_OK) {
		return MAT_BUF_ALLOC_FAIL;
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

	return MAT_BUF_ALLOC_OK;
}


int algebraTests_transposeSwap(matrix_t *M)
{
	if (algebraTests_realTrp(M) != MAT_BUF_ALLOC_OK) {
		return MAT_BUF_ALLOC_FAIL;
	}
	matrix_trp(M);

	return MAT_BUF_ALLOC_OK;
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


int algebraTests_matrixAllocable(unsigned int rows, unsigned int cols)
{
	float *help;

	if (SIZE_MAX / rows < cols) {
		return CHECK_FAIL;
	}

	help = calloc(rows * cols, sizeof(float));
	if (help == NULL) {
		return CHECK_FAIL;
	}

	free(help);
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


int algebraTests_dataTrpCheck(matrix_t *M1, matrix_t *M2)
{
	unsigned int row, col, nRows, nCols;

	nRows = matrix_rowsGet(M1);
	nCols = matrix_colsGet(M1);

	if (nRows != matrix_colsGet(M2) || nCols != matrix_rowsGet(M2)) {
		return CHECK_FAIL;
	}

	for (row = 0; row < nRows; row++) {
		for (col = 0; col < nCols; col++) {
			if (*matrix_at(M1, row, col) != *matrix_at(M2, col, row)) {
				return CHECK_FAIL;
			}
		}
	}

	return CHECK_OK;
}


int algebraTest_equalMatrix(const matrix_t *M1, const matrix_t *M2)
{
	unsigned int i;

	if (M1->rows != M2->rows || M1->cols != M2->cols) {
		return CHECK_FAIL;
	}

	if (M1->transposed != M2->transposed) {
		return CHECK_FAIL;
	}

	for (i = 0; i < M1->rows * M1->cols; i++) {
		if (M1->data[i] != M2->data[i]) {
			return CHECK_FAIL;
		}
	}

	return CHECK_OK;
}


int algebraTests_submatCheck(const matrix_t *dst, unsigned int row, unsigned int col, const matrix_t *src, const matrix_t *M)
{
	unsigned int currRow, currCol;
	int inSubmat;

	if (dst->transposed != 0 || src->transposed != 0 || M->transposed != 0) {
		return CHECK_FAIL;
	}

	if (dst->rows != M->rows || dst->cols != M->cols) {
		return CHECK_FAIL;
	}

	if (col + src->cols > dst->cols || row + src->rows > dst->rows) {
		return CHECK_FAIL;
	}

	for (currRow = 0; currRow < dst->rows; currRow++) {
		for (currCol = 0; currCol < dst->cols; currCol++) {
			inSubmat = currRow >= row && currRow < row + src->rows && currCol >= col && currCol < col + src->cols;

			if (inSubmat) {
				/* Checking elements from src */
				if (*matrix_at(M, currRow, currCol) != *matrix_at(src, currRow - row, currCol - col)) {
					return CHECK_FAIL;
				}
			}
			else {
				/* Checking elements from dst */
				if (*matrix_at(M, currRow, currCol) != *matrix_at(dst, currRow, currCol)) {
					return CHECK_FAIL;
				}
			}
		}
	}

	return CHECK_OK;
}
