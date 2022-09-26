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

#include <unity_fixture.h>

#include <matrix.h>

#include <stdlib.h>

#include "../tools.h"
#include "../buffs.h"


#define DELTA 1e-5


static matrix_t M1, M2, M3, Expected;

static float *buf = NULL;
static int bufLen;


/* ##############################################################################
 * ----------------------        matrix_inv tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_inv_stdMat);


TEST_SETUP(group_matrix_inv_stdMat)
{
	/* M1 = A */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));

	/* Expected = A^(-1) */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsInvA, buffs_colsInvA, buffs_invA, buffs_colsInvA * buffs_rowsInvA));

	/* Allocating result matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, Expected.rows, Expected.cols));

	bufLen = M1.rows * M1.cols * 2;
	buf = malloc(sizeof(float) * bufLen);
	TEST_ASSERT_NOT_NULL(buf);

	M3.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_inv_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Expected);

	free(buf);
	buf = NULL;
}


TEST(group_matrix_inv_stdMat, matrix_inv_std)
{
	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);
}


TEST(group_matrix_inv_stdMat, matrix_inv_firstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);
}


TEST(group_matrix_inv_stdMat, matrix_inv_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);
}


TEST(group_matrix_inv_stdMat, matrix_inv_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);
}


TEST(group_matrix_inv_stdMat, matrix_inv_InPlaceInv)
{
	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M1, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M1);
}


TEST(group_matrix_inv_stdMat, matrix_inv_InPlaceInvTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M1, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M1);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_inv_stdMat, matrix_inv_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M3, &M1));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(M3, M1);
}


/* In this test group we are checking if A * A^(-1) = I */
TEST_GROUP(group_matrix_inv_bigMat);


TEST_SETUP(group_matrix_inv_bigMat)
{
	/* M1 = J */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsJ, buffs_colsJ, buffs_J, buffs_colsJ * buffs_rowsJ));

	/* Allocating result matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, M1.rows, M1.cols));

	/* Allocating temporary matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M1.cols));

	bufLen = M1.rows * M1.cols * 2;
	buf = malloc(sizeof(float) * bufLen);
	TEST_ASSERT_NOT_NULL(buf);

	/* Expected is identity matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, M1.rows, M1.cols));
	matrix_diag(&Expected);
}


TEST_TEAR_DOWN(group_matrix_inv_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Expected);

	free(buf);
	buf = NULL;
}


TEST(group_matrix_inv_bigMat, matrix_inv_bigMat)
{
	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_INT(MAT_PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_inv_bigMat, matrix_inv_bigMatFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_INT(MAT_PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_inv_bigMat, matrix_inv_bigMatResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_INT(MAT_PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_inv_bigMat, matrix_inv_bigMatAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_INT(MAT_PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


/* This group contains tests with individual matrices which can be hard to inverse */
TEST_GROUP(group_matrix_inv_otherMats);


/* All tests will allocate it's own matrices */
TEST_SETUP(group_matrix_inv_otherMats)
{
	M1.data = NULL;
	M2.data = NULL;
	Expected.data = NULL;

	buf = NULL;
}


TEST_TEAR_DOWN(group_matrix_inv_otherMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&Expected);

	free(buf);
	buf = NULL;
}


TEST(group_matrix_inv_otherMats, matrix_inv_zeroOnDiag)
{
	/*
		Currently we are using algorithm to inverse matrix, which fails when finds zero on main diagonal
		even if mathematically it is possible to inverse this matrix.
		Such a case is presented in this test. It has been wrote for the future when we decide to use another method.
		Reference: https://github.com/phoenix-pilot/phoenix-pilot-core/issues/110
	*/
	TEST_IGNORE();

	/* M1 = K */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsK, buffs_colsA, buffs_K, buffs_colsK * buffs_rowsK));

	/* Expected = K^(-1) */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsInvK, buffs_colsInvK, buffs_invK, buffs_colsInvK * buffs_rowsInvK));

	/* Allocating result matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, Expected.rows, Expected.cols));

	bufLen = M1.rows * M1.cols * 2;
	buf = malloc(sizeof(float) * bufLen);
	TEST_ASSERT_NOT_NULL(buf);

	/* Without transposition */
	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);

	matrix_zeroes(&M2);
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	/* With transposition */
	TEST_ASSERT_EQUAL_INT(MAT_INV_OK, matrix_inv(&M1, &M2, buf, bufLen));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M2);
}


TEST_GROUP(group_matrix_inv_badMat);


TEST_SETUP(group_matrix_inv_badMat)
{
	/* M1 = J, J is invertible. Tests will modify it in different way so that it is incorrect */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsJ, buffs_colsJ, buffs_J, buffs_colsJ * buffs_rowsJ));

	/* Allocating result matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, M1.rows, M1.cols));

	bufLen = M1.rows * M1.cols * 2;
	buf = malloc(sizeof(float) * bufLen);
	TEST_ASSERT_NOT_NULL(buf);

	M3.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_inv_badMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);

	free(buf);
	buf = NULL;
}


TEST(group_matrix_inv_badMat, matrix_inv_detIsZero)
{
	int col, rowsNum, colsNum;

	rowsNum = matrix_rowsGet(&M1);
	colsNum = matrix_colsGet(&M1);

	/* Copying first row to the middle row */
	for (col = 0; col < colsNum; col++) {
		*matrix_at(&M1, rowsNum / 2, col) = *matrix_at(&M1, 0, col);
	}

	/* Now `M1` has two rows which are the same, so det(M1) = 0 */
	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));
}


TEST(group_matrix_inv_badMat, matrix_inv_notSqrMat)
{
	/* Too few columns */
	M1.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	/* Too few rows */
	M1.cols++;
	M1.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));
}


TEST(group_matrix_inv_badMat, matrix_inv_badResMat)
{
	/* Too few columns */
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	/* Too few rows */
	M2.cols++;
	M2.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));

	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen));
}


TEST(group_matrix_inv_badMat, matrix_inv_tooSmallBuf)
{
	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen / 2));
}


TEST(group_matrix_inv_badMat, matrix_inv_failureRetain)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M3, &M2));

	TEST_ASSERT_EQUAL_INT(MAT_INV_FAIL, matrix_inv(&M1, &M2, buf, bufLen / 2));

	TEST_ASSERT_EQUAL_MATRIX(M3, M2);
}


TEST_GROUP_RUNNER(group_matrix_inv)
{
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_std);
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_firstMatTrp);
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_resultMatTrp);
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_allMatTrp);
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_InPlaceInv);
	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_InPlaceInvTrp);

	RUN_TEST_CASE(group_matrix_inv_stdMat, matrix_inv_sourceRetain);

	RUN_TEST_CASE(group_matrix_inv_bigMat, matrix_inv_bigMat);
	RUN_TEST_CASE(group_matrix_inv_bigMat, matrix_inv_bigMatFirstMatTrp);
	RUN_TEST_CASE(group_matrix_inv_bigMat, matrix_inv_bigMatResultMatTrp);
	RUN_TEST_CASE(group_matrix_inv_bigMat, matrix_inv_bigMatAllMatTrp);

	RUN_TEST_CASE(group_matrix_inv_otherMats, matrix_inv_zeroOnDiag);

	RUN_TEST_CASE(group_matrix_inv_badMat, matrix_inv_detIsZero);
	RUN_TEST_CASE(group_matrix_inv_badMat, matrix_inv_notSqrMat);
	RUN_TEST_CASE(group_matrix_inv_badMat, matrix_inv_badResMat);
	RUN_TEST_CASE(group_matrix_inv_badMat, matrix_inv_tooSmallBuf);
	RUN_TEST_CASE(group_matrix_inv_badMat, matrix_inv_failureRetain);
}
