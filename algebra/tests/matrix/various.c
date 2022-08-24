/*
 * Phoenix-Pilot
 *
 * Unit tests for matrix library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <string.h>

#include <unity_fixture.h>

#include <matrix.h>

#include "tools.h"

/* Creating matrices for testing */

/* ROWS and COLS must be at least 2 */
#define ROWS 10
#define COLS 5

static float buf[ROWS * COLS];
static matrix_t stMat = { .data = buf, .cols = COLS, .rows = ROWS, .transposed = 0 };

/* Matrix for dynamical allocation */
static matrix_t M;

/* ##############################################################################
 * -----------------------        matrix_trp tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_trp);


TEST_SETUP(group_matrix_trp)
{
	stMat.transposed = 0;
}


TEST_TEAR_DOWN(group_matrix_trp)
{
	stMat.transposed = 0;
}


TEST(group_matrix_trp, matrix_trp_std)
{
	TEST_ASSERT_FALSE(stMat.transposed);

	matrix_trp(&stMat);
	TEST_ASSERT_TRUE(stMat.transposed);

	matrix_trp(&stMat);
	TEST_ASSERT_FALSE(stMat.transposed);
}


TEST_GROUP_RUNNER(group_matrix_trp)
{
	RUN_TEST_CASE(group_matrix_trp, matrix_trp_std);
}


/* ##############################################################################
 * ---------------------        matrix_zeroes tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_zeroes);


TEST_SETUP(group_matrix_zeroes)
{
	memset(buf, 1, sizeof(float) * ROWS * COLS);
	M.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_zeroes)
{
	matrix_bufFree(&M);
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


TEST(group_matrix_zeroes, matrix_zeroes_std)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M, ROWS, COLS));
	algebraTests_fillWithVal(&M, 1.0);

	matrix_zeroes(&M);

	TEST_ASSERT_EQUAL_INT(ROWS, M.rows);
	TEST_ASSERT_EQUAL_INT(COLS, M.cols);
	TEST_ASSERT_FALSE(M.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_checkMatrixZeroes(&M));
}


TEST(group_matrix_zeroes, matrix_zeroes_stdTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M, ROWS, COLS));
	algebraTests_fillWithVal(&M, 1.0);
	matrix_trp(&M);

	matrix_zeroes(&M);

	TEST_ASSERT_EQUAL_INT(ROWS, M.rows);
	TEST_ASSERT_EQUAL_INT(COLS, M.cols);
	TEST_ASSERT_TRUE(M.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_checkMatrixZeroes(&M));
}


TEST(group_matrix_zeroes, matrix_zeroes_stMat)
{
	matrix_zeroes(&stMat);

	TEST_ASSERT_EQUAL_INT(ROWS, stMat.rows);
	TEST_ASSERT_EQUAL_INT(COLS, stMat.cols);
	TEST_ASSERT_FALSE(stMat.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_checkMatrixZeroes(&stMat));
}


TEST_GROUP_RUNNER(group_matrix_zeroes)
{
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_std);
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_stdTrp);
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_stMat);
}
