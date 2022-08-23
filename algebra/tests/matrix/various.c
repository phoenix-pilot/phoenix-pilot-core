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

/* Creating matrix for testing */

/* ROWS and COLS must be at least 2 */
#define ROWS 10
#define COLS 5

static float buf[ROWS * COLS];
static matrix_t stMat = { .data = buf, .cols = COLS, .rows = ROWS, .transposed = 0 };


/* ##############################################################################
 * ---------------------        matrix_zeroes tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_zeroes);


TEST_SETUP(group_matrix_zeroes)
{
	memset(buf, 1, sizeof(float) * ROWS * COLS);
}


TEST_TEAR_DOWN(group_matrix_zeroes)
{
}


TEST(group_matrix_zeroes, matrix_zeroes_std)
{
	int row, col;

	matrix_zeroes(&stMat);

	TEST_ASSERT_EQUAL_INT(ROWS, stMat.rows);
	TEST_ASSERT_EQUAL_INT(COLS, stMat.cols);
	TEST_ASSERT_FALSE(stMat.transposed);

	for (row = 0; row < stMat.rows; row++) {
		for (col = 0; col < stMat.cols; col++) {
			TEST_ASSERT_EQUAL_FLOAT(0.0, *matrix_at(&stMat, row, col));
		}
	}
}


TEST_GROUP_RUNNER(group_matrix_zeroes)
{
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_std);
}
