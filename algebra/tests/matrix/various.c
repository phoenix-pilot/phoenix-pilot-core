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

#include "tools.h"
#include "buffs.h"


/* Creating matrices for testing */

/* ROWS and COLS must be at least 2 */
#define ROWS 10
#define COLS 5

/* Size of square matrices */
#define SQUARE_MAT_SIZE 5

static float buf[ROWS * COLS];
static matrix_t stMat = { .data = buf, .cols = COLS, .rows = ROWS, .transposed = 0 };

/* Matrix for dynamical allocation */
static matrix_t M1, M2;


/* Must be different than zero and one */
static float initVal[] = { 2.0 };
static int initValLen = 1;

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
	M1.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_zeroes)
{
	matrix_bufFree(&M1);
}


TEST(group_matrix_zeroes, matrix_zeroes_std)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, ROWS, COLS));
	algebraTests_buffFill(&M1, initVal, initValLen);

	matrix_zeroes(&M1);

	TEST_ASSERT_EQUAL_INT(ROWS, M1.rows);
	TEST_ASSERT_EQUAL_INT(COLS, M1.cols);
	TEST_ASSERT_FALSE(M1.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_matrixZeroesCheck(&M1));
}


TEST(group_matrix_zeroes, matrix_zeroes_stdTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, ROWS, COLS));
	algebraTests_buffFill(&M1, initVal, initValLen);
	matrix_trp(&M1);

	matrix_zeroes(&M1);

	TEST_ASSERT_EQUAL_INT(ROWS, M1.rows);
	TEST_ASSERT_EQUAL_INT(COLS, M1.cols);
	TEST_ASSERT_TRUE(M1.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_matrixZeroesCheck(&M1));
}


TEST(group_matrix_zeroes, matrix_zeroes_stMat)
{
	matrix_zeroes(&stMat);

	TEST_ASSERT_EQUAL_INT(ROWS, stMat.rows);
	TEST_ASSERT_EQUAL_INT(COLS, stMat.cols);
	TEST_ASSERT_FALSE(stMat.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_matrixZeroesCheck(&stMat));
}


TEST_GROUP_RUNNER(group_matrix_zeroes)
{
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_std);
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_stdTrp);
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_stMat);
}


/* ##############################################################################
 * -----------------------        matrix_diag tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_diag);


TEST_SETUP(group_matrix_diag)
{
	M1.data = NULL;
	M1.transposed = 0;
}


TEST_TEAR_DOWN(group_matrix_diag)
{
	matrix_bufFree(&M1);
	M1.transposed = 0;
}


TEST(group_matrix_diag, matrix_diag_squareMat)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	algebraTests_buffFill(&M1, initVal, initValLen);

	matrix_diag(&M1);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_diagCheck(&M1));
}


TEST(group_matrix_diag, matrix_diag_squareMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	matrix_trp(&M1);
	algebraTests_buffFill(&M1, initVal, initValLen);

	matrix_diag(&M1);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_diagCheck(&M1));
}


TEST(group_matrix_diag, matrix_diag_notSquareMat)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, ROWS, COLS));
	algebraTests_buffFill(&M1, initVal, initValLen);

	matrix_diag(&M1);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_diagCheck(&M1));
}


TEST(group_matrix_diag, matrix_diag_notSquareMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, ROWS, COLS));
	matrix_trp(&M1);
	algebraTests_buffFill(&M1, initVal, initValLen);

	matrix_diag(&M1);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_diagCheck(&M1));
}


TEST_GROUP_RUNNER(group_matrix_diag)
{
	RUN_TEST_CASE(group_matrix_diag, matrix_diag_squareMat);
	RUN_TEST_CASE(group_matrix_diag, matrix_diag_squareMatTrp);
	RUN_TEST_CASE(group_matrix_diag, matrix_diag_notSquareMat);
	RUN_TEST_CASE(group_matrix_diag, matrix_diag_notSquareMatTrp);
}


/* ##############################################################################
 * ---------------------        matrix_times tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_times);


TEST_SETUP(group_matrix_times)
{
	M1.data = NULL;
	M2.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_times)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
}


TEST(group_matrix_times, matrix_times_std)
{
	int i;
	float scalar = -123.45;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, scalar);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(scalar * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_stdTrp)
{
	int i;
	float scalar = -98.76;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, scalar);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(scalar * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_bigMat)
{
	int i;
	float scalar = 123.45;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, scalar);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(scalar * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_bigMatTrp)
{
	int i;
	float scalar = -98.76;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, scalar);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(scalar * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_inf)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, INFINITY);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(INFINITY * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_infTrp)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, INFINITY);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(INFINITY * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_minusInf)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, -INFINITY);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(-INFINITY * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_minusInfTrp)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, -INFINITY);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(-INFINITY * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_nan)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, NAN);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(NAN * M2.data[i], M1.data[i]);
	}
}


TEST(group_matrix_times, matrix_times_nanTrp)
{
	int i;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_times(&M1, NAN);

	TEST_ASSERT_EQUAL_UINT(M2.rows, M1.rows);
	TEST_ASSERT_EQUAL_UINT(M2.cols, M1.cols);
	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	for (i = 0; i < M1.rows * M1.cols; i++) {
		TEST_ASSERT_EQUAL_FLOAT(NAN * M2.data[i], M1.data[i]);
	}
}


TEST_GROUP_RUNNER(group_matrix_times)
{
	RUN_TEST_CASE(group_matrix_times, matrix_times_std);
	RUN_TEST_CASE(group_matrix_times, matrix_times_stdTrp);
	RUN_TEST_CASE(group_matrix_times, matrix_times_bigMat);
	RUN_TEST_CASE(group_matrix_times, matrix_times_bigMatTrp);
	RUN_TEST_CASE(group_matrix_times, matrix_times_inf);
	RUN_TEST_CASE(group_matrix_times, matrix_times_infTrp);
	RUN_TEST_CASE(group_matrix_times, matrix_times_minusInf);
	RUN_TEST_CASE(group_matrix_times, matrix_times_minusInfTrp);
	RUN_TEST_CASE(group_matrix_times, matrix_times_nan);
	RUN_TEST_CASE(group_matrix_times, matrix_times_nanTrp);
}
