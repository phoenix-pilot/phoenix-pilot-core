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
#include <stdlib.h>

#include <unity_fixture.h>

#include <matrix.h>


/* Consts used by tests */
#define SMALL_SHIFT 1
#define BIG_SHIFT   1234

#define BUF_ALLOC_OK   0
#define BUF_ALLOC_FAIL -1


/* Creating matrices for testing */

/* ROWS and COLS must be at least 2 */
#define ROWS 10
#define COLS 5

static float buf[ROWS * COLS];

/* Statically allocated matrix */
static matrix_t stMat = { .data = buf, .cols = COLS, .rows = ROWS, .transposed = 0 };

/* Matrix for dynamic allocations */
static matrix_t dynMat;


void algebraTests_checkInvalidSeek(matrix_t *M)
{
	int row, col;

	/* Both row and col outside matrix */
	TEST_ASSERT_NULL(matrix_at(M, M->rows, M->cols));
	TEST_ASSERT_NULL(matrix_at(M, M->rows + SMALL_SHIFT, M->cols + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(M, M->rows + BIG_SHIFT, M->cols + BIG_SHIFT));

	/* Only row outside matrix */
	col = M->cols / 2;
	TEST_ASSERT_NULL(matrix_at(M, M->rows, col));
	TEST_ASSERT_NULL(matrix_at(M, M->rows + SMALL_SHIFT, col));
	TEST_ASSERT_NULL(matrix_at(M, M->rows + BIG_SHIFT, col));

	/* Only col outside matrix */
	row = M->rows / 2;
	TEST_ASSERT_NULL(matrix_at(M, row, M->cols));
	TEST_ASSERT_NULL(matrix_at(M, row, M->cols + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(M, row, M->cols + BIG_SHIFT));
}


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
	algebraTests_checkInvalidSeek(&stMat);
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


/* ##############################################################################
 * -------------------        matrix_bufAlloc tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_bufAlloc);


TEST_SETUP(group_matrix_bufAlloc)
{
	dynMat.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_bufAlloc)
{
	free(dynMat.data);
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_structElems)
{
	dynMat.rows = 0;
	dynMat.cols = 0;
	dynMat.transposed = 1;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	TEST_ASSERT_EQUAL_INT(ROWS, dynMat.rows);
	TEST_ASSERT_EQUAL_INT(COLS, dynMat.cols);
	TEST_ASSERT_FALSE(dynMat.transposed);
	TEST_ASSERT_NOT_NULL(dynMat.data);
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_validSeek)
{
	int row, col;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_NOT_NULL(matrix_at(&dynMat, row, col));
		}
	}
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_invalidSeek)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	algebraTests_checkInvalidSeek(&dynMat);
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_initVal)
{
	int row, col;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_EQUAL_FLOAT(0, *matrix_at(&dynMat, row, col));
		}
	}
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_readAndWrite)
{
	int row, col, i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	/* Assigning different value to every matrix element */
	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			*matrix_at(&dynMat, row, col) = i;
			i++;
		}
	}

	/* Checking every matrix element */
	i = 0;
	for (row = 0; row < ROWS; row++) {
		for (col = 0; col < COLS; col++) {
			TEST_ASSERT_EQUAL_FLOAT(i, *matrix_at(&dynMat, row, col));
			i++;
		}
	}
}


TEST(group_matrix_bufAlloc, matrix_bufAlloc_invalidArgs)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_FAIL, matrix_bufAlloc(&dynMat, 0, COLS));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_FAIL, matrix_bufAlloc(&dynMat, ROWS, 0));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_FAIL, matrix_bufAlloc(&dynMat, sqrt(UINT_MAX), sqrt(UINT_MAX)));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_FAIL, matrix_bufAlloc(&dynMat, UINT_MAX, UINT_MAX));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_FAIL, matrix_bufAlloc(&dynMat, 0, 0));
}


TEST_GROUP_RUNNER(group_matrix_bufAlloc)
{
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_structElems);
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_validSeek);
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_invalidSeek);
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_initVal);
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_readAndWrite);
	RUN_TEST_CASE(group_matrix_bufAlloc, matrix_bufAlloc_invalidArgs);
}
