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


/* Consts used by tests */
#define SMALL_SHIFT 1
#define BIG_SHIFT   1234


/* Creating matrices for testing */

/* ROWS and COLS must be at least 2 */
#define ROWS 10
#define COLS 5

static float buf[ROWS * COLS];

/* Statically allocated matrix */
static matrix_t stMat = { .data = buf, .cols = COLS, .rows = ROWS, .transposed = 0 };


/* ##############################################################################
 * ----------------------        matrix_at tests       --------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_at);

TEST_SETUP(group_matrix_at)
{
	memset(buf, 1, sizeof(float) * ROWS * COLS);
}


TEST_TEAR_DOWN(group_matrix_at)
{
}


TEST(group_matrix_at, matrix_at_validSeek)
{
	int row, col;

	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_NOT_NULL(matrix_at(&stMat, row, col));
		}
	}
}


TEST(group_matrix_at, matrix_at_invalidSeek)
{
	int row, col;

	/* Both row and col outside matrix */
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS, COLS));
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS + SMALL_SHIFT, COLS + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS + BIG_SHIFT, COLS + BIG_SHIFT));

	/* Only row outside matrix */
	col = COLS / 2;
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS, col));
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS + SMALL_SHIFT, col));
	TEST_ASSERT_NULL(matrix_at(&stMat, ROWS + BIG_SHIFT, col));

	/* Only col outside matrix */
	row = ROWS / 2;
	TEST_ASSERT_NULL(matrix_at(&stMat, row, COLS));
	TEST_ASSERT_NULL(matrix_at(&stMat, row, COLS + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(&stMat, row, COLS + BIG_SHIFT));
}


TEST(group_matrix_at, matrix_at_writing)
{
	int row, col, i;
	float *matElem;
	float exp_data[ROWS * COLS];

	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			matElem = matrix_at(&stMat, row, col);
			TEST_ASSERT_NOT_NULL(matElem);

			/* Assigning different value to every matrix element and logically corresponding position in exp_data */
			*matElem = exp_data[row * COLS + col] = i;
			i++;
		}
	}

	TEST_ASSERT_EQUAL_FLOAT_ARRAY(exp_data, stMat.data, ROWS * COLS);
}


TEST(group_matrix_at, matrix_at_reading)
{
	int row, col, i;

	/* Assigning different value to every matrix element */
	for (i = 0; i < ROWS * COLS; i++) {
		stMat.data[i] = i;
	}

	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_EQUAL_FLOAT(i, *matrix_at(&stMat, row, col));
			i++;
		}
	}
}


TEST(group_matrix_at, matrix_at_writingAndReading)
{
	int row, col, i;

	/* Assigning different value to every matrix element */
	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			*matrix_at(&stMat, row, col) = i;
			i++;
		}
	}

	/* Checking every matrix element */
	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_EQUAL_FLOAT(i, *matrix_at(&stMat, row, col));
			i++;
		}
	}
}


TEST_GROUP_RUNNER(group_matrix_at)
{
	RUN_TEST_CASE(group_matrix_at, matrix_at_validSeek);
	RUN_TEST_CASE(group_matrix_at, matrix_at_invalidSeek);
	RUN_TEST_CASE(group_matrix_at, matrix_at_writing);
	RUN_TEST_CASE(group_matrix_at, matrix_at_reading);
	RUN_TEST_CASE(group_matrix_at, matrix_at_writingAndReading);
}
