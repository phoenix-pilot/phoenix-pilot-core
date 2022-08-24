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
	int rowsNum, colsNum, row, col;

	if (M->transposed) {
		rowsNum = M->cols;
		colsNum = M->rows;
	}
	else {
		rowsNum = M->rows;
		colsNum = M->cols;
	}

	/* Both row and col outside matrix */
	TEST_ASSERT_NULL(matrix_at(M, rowsNum, colsNum));
	TEST_ASSERT_NULL(matrix_at(M, rowsNum + SMALL_SHIFT, colsNum + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(M, rowsNum + BIG_SHIFT, colsNum + BIG_SHIFT));

	/* Only row outside matrix */
	col = colsNum / 2; /* arbitrary position within columns */
	TEST_ASSERT_NULL(matrix_at(M, rowsNum, col));
	TEST_ASSERT_NULL(matrix_at(M, rowsNum + SMALL_SHIFT, col));
	TEST_ASSERT_NULL(matrix_at(M, rowsNum + BIG_SHIFT, col));

	/* Only col outside matrix */
	row = rowsNum / 2; /* arbitrary position within rows */
	TEST_ASSERT_NULL(matrix_at(M, row, colsNum));
	TEST_ASSERT_NULL(matrix_at(M, row, colsNum + SMALL_SHIFT));
	TEST_ASSERT_NULL(matrix_at(M, row, colsNum + BIG_SHIFT));
}


/* ##############################################################################
 * ----------------------        matrix_at tests       --------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_at);

TEST_SETUP(group_matrix_at)
{
	memset(buf, 1, sizeof(float) * ROWS * COLS);
	stMat.transposed = 0;
}


TEST_TEAR_DOWN(group_matrix_at)
{
	stMat.transposed = 0;
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


TEST(group_matrix_at, matrix_at_validSeekTrp)
{
	int row, col;

	matrix_trp(&stMat);

	for (row = 0; row < COLS; row++) {
		for (col = 0; col < ROWS; col++) {
			TEST_ASSERT_NOT_NULL(matrix_at(&stMat, row, col));
		}
	}
}


TEST(group_matrix_at, matrix_at_invalidSeek)
{
	algebraTests_checkInvalidSeek(&stMat);
}


TEST(group_matrix_at, matrix_at_invalidSeekTrp)
{
	matrix_trp(&stMat);
	algebraTests_checkInvalidSeek(&stMat);
}


TEST(group_matrix_at, matrix_at_write)
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


TEST(group_matrix_at, matrix_at_writeTrp)
{
	int row, col, i;
	float *matElem;
	float exp_data[ROWS * COLS];

	matrix_trp(&stMat);

	i = 0;
	for (col = 0; col < ROWS; col++) {
		for (row = 0; row < COLS; row++) {
			matElem = matrix_at(&stMat, row, col);
			TEST_ASSERT_NOT_NULL(matElem);

			/* Assigning different value to every matrix element and logically corresponding position in exp_data */
			*matElem = exp_data[col * COLS + row] = i;
			i++;
		}
	}

	TEST_ASSERT_EQUAL_FLOAT_ARRAY(exp_data, stMat.data, ROWS * COLS);
}


TEST(group_matrix_at, matrix_at_read)
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


TEST(group_matrix_at, matrix_at_readTrp)
{
	int row, col, i;

	for (i = 0; i < ROWS * COLS; i++) {
		stMat.data[i] = i;
	}

	matrix_trp(&stMat);

	i = 0;
	for (col = 0; col < ROWS; col++) {
		for (row = 0; row < COLS; row++) {
			TEST_ASSERT_EQUAL_FLOAT(i, *matrix_at(&stMat, row, col));
			i++;
		}
	}
}


TEST(group_matrix_at, matrix_at_writeRead)
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


TEST(group_matrix_at, matrix_at_writeReadTrp)
{
	int row, col, i;

	matrix_trp(&stMat);

	/* Assigning different value to every matrix element */
	i = 0;
	for (row = 0; row < COLS; row++) {
		for (col = 0; col < ROWS; col++) {
			*matrix_at(&stMat, row, col) = i;
			i++;
		}
	}

	/* Checking every matrix element */
	i = 0;
	for (row = 0; row < COLS; row++) {
		for (col = 0; col < ROWS; col++) {
			TEST_ASSERT_EQUAL_FLOAT(i, *matrix_at(&stMat, row, col));
			i++;
		}
	}
}


TEST_GROUP_RUNNER(group_matrix_at)
{
	RUN_TEST_CASE(group_matrix_at, matrix_at_validSeek);
	RUN_TEST_CASE(group_matrix_at, matrix_at_validSeekTrp);
	RUN_TEST_CASE(group_matrix_at, matrix_at_invalidSeek);
	RUN_TEST_CASE(group_matrix_at, matrix_at_invalidSeekTrp);
	RUN_TEST_CASE(group_matrix_at, matrix_at_write);
	RUN_TEST_CASE(group_matrix_at, matrix_at_writeTrp);
	RUN_TEST_CASE(group_matrix_at, matrix_at_read);
	RUN_TEST_CASE(group_matrix_at, matrix_at_readTrp);
	RUN_TEST_CASE(group_matrix_at, matrix_at_writeRead);
	RUN_TEST_CASE(group_matrix_at, matrix_at_writeReadTrp);
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


/* ##############################################################################
 * ---------------------        matrix_bufFree tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_bufFree);


TEST_SETUP(group_matrix_bufFree)
{
	dynMat.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_bufFree)
{
	free(dynMat.data);
}


TEST(group_matrix_bufFree, matrix_bufFree_std)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	matrix_bufFree(&dynMat);

	TEST_ASSERT_NULL(dynMat.data);
}


TEST(group_matrix_bufFree, matrix_bufFree_doubleFreeSafe)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&dynMat, ROWS, COLS));

	matrix_bufFree(&dynMat);
	TEST_ASSERT_NULL(dynMat.data);

	matrix_bufFree(&dynMat);
	TEST_ASSERT_NULL(dynMat.data);
}


TEST_GROUP_RUNNER(group_matrix_bufFree)
{
	RUN_TEST_CASE(group_matrix_bufFree, matrix_bufFree_std);
	RUN_TEST_CASE(group_matrix_bufFree, matrix_bufFree_doubleFreeSafe);
}
