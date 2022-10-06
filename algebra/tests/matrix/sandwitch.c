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

#include "../tools.h"
#include "buffs.h"


#define DELTA 1

/* Must be bigger than 1 */
#define SQUARE_MAT_SIZE 4


static matrix_t M1, M2, M3, M4, M5, Expected, tmp;


/* ##############################################################################
 * -------------------        matrix_sandwitch tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_sandwitch_stdMat);


TEST_SETUP(group_matrix_sandwitch_stdMat)
{
	/* M1 = A */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));

	/* M2 = B */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsB, buffs_colsB, buffs_B, buffs_colsB * buffs_rowsB));

	/* Expected = A * B * A^T */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsAsandB, buffs_colsAsandB, buffs_AsandB, buffs_colsAsandB * buffs_rowsAsandB));

	/* Allocating matrix for results and filling with non zero data */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, Expected.rows, Expected.cols));
	algebraTests_buffFill(&M3, initVal, initValLen);

	/* Allocating temporary matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));
}


TEST_TEAR_DOWN(group_matrix_sandwitch_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Expected);
	matrix_bufFree(&tmp);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_std)
{
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sandwitch_stdMat, matrix_sandwitch_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST_GROUP(group_matrix_sandwitch_bigMat);


TEST_SETUP(group_matrix_sandwitch_bigMat)
{
	/* M1 = G */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsG, buffs_colsG, buffs_G, buffs_colsG * buffs_rowsG));

	/* M2 = H */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsH, buffs_colsH, buffs_H, buffs_colsH * buffs_rowsH));

	/* Expected = G * H * G^T */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsGsandH, buffs_colsGsandH, buffs_GsandH, buffs_colsGsandH * buffs_rowsGsandH));

	/* Allocating matrix for results and filling with non zero data */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, Expected.rows, Expected.cols));
	algebraTests_buffFill(&M3, initVal, initValLen);

	/* Allocating temporary matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));

	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sandwitch_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&Expected);
	matrix_bufFree(&tmp);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_sandwitch_bigMat, matrix_sandwitch_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(M4, M1);
	TEST_ASSERT_EQUAL_MATRIX(M5, M2);
}


TEST_GROUP(group_matrix_sandwitch_badMats);


TEST_SETUP(group_matrix_sandwitch_badMats)
{
	/* These matrix sizes are correct, but will be changed in tests */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M2.cols));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));
	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sandwitch_badMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&tmp);
}


TEST(group_matrix_sandwitch_badMats, matrix_sandwitch_badInputMats)
{
	M2.rows--;
	M2.cols--;

	/* We want tmp to have right size */
	tmp.cols--;

	/* No matrix is transposed */
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	/* First matrix is transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	/* Second matrix is transposed */
	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	/* First and second transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));
}


TEST(group_matrix_sandwitch_badMats, matrix_sandwitch_badResMat)
{
	/* Incorrect rows number */
	M3.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	/* Incorrect cols number */
	M3.rows++;
	M3.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));
}


TEST(group_matrix_sandwitch_badMats, matrix_sandwitch_badTmpMat)
{
	/* Incorrect rows number */
	tmp.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	/* Incorrect cols number */
	tmp.rows++;
	tmp.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));
}


/* This test checks if result matrix and temporary are changing when function fails */
TEST(group_matrix_sandwitch_badMats, matrix_sandwitch_failureRetain)
{
	M2.rows--;
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M3));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &tmp));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(M4, M3);
	TEST_ASSERT_EQUAL_MATRIX(M5, tmp);
}


TEST_GROUP_RUNNER(group_matrix_sandwitch)
{
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_std);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_firstMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_secondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_firstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_resultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_stdMat, matrix_sandwitch_allMatTrp);

	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsStd);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsFirstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsResultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_bigMatsAllMatTrp);

	RUN_TEST_CASE(group_matrix_sandwitch_bigMat, matrix_sandwitch_sourceRetain);

	RUN_TEST_CASE(group_matrix_sandwitch_badMats, matrix_sandwitch_badInputMats);
	RUN_TEST_CASE(group_matrix_sandwitch_badMats, matrix_sandwitch_badResMat);
	RUN_TEST_CASE(group_matrix_sandwitch_badMats, matrix_sandwitch_badTmpMat);
	RUN_TEST_CASE(group_matrix_sandwitch_badMats, matrix_sandwitch_failureRetain);
}


/* ##############################################################################
 * -------------------        matrix_sparseSandwitch tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_sparseSandwitch_stdMat);


TEST_SETUP(group_matrix_sparseSandwitch_stdMat)
{
	/* M1 = A */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));

	/* M2 = B */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsB, buffs_colsB, buffs_B, buffs_colsB * buffs_rowsB));

	/* Expected = A * B * A^T */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsAsandB, buffs_colsAsandB, buffs_AsandB, buffs_colsAsandB * buffs_rowsAsandB));

	/* Allocating matrix for results and filling with non zero data */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, Expected.rows, Expected.cols));
	algebraTests_buffFill(&M3, initVal, initValLen);

	/* Allocating temporary matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));
}


TEST_TEAR_DOWN(group_matrix_sparseSandwitch_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Expected);
	matrix_bufFree(&tmp);
}

TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_std)
{
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M3);
}


TEST_GROUP(group_matrix_sparseSandwitch_bigMat);


TEST_SETUP(group_matrix_sparseSandwitch_bigMat)
{
	/* M1 = G */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsG, buffs_colsG, buffs_G, buffs_colsG * buffs_rowsG));

	/* M2 = H */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsH, buffs_colsH, buffs_H, buffs_colsH * buffs_rowsH));

	/* Expected = G * H * G^T */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&Expected, buffs_rowsGsandH, buffs_colsGsandH, buffs_GsandH, buffs_colsGsandH * buffs_rowsGsandH));

	/* Allocating matrix for results and filling with non zero data */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, Expected.rows, Expected.cols));
	algebraTests_buffFill(&M3, initVal, initValLen);

	/* Allocating temporary matrix */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));

	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sparseSandwitch_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&Expected);
	matrix_bufFree(&tmp);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_MATRIX_WITHIN(DELTA, Expected, M3);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M1));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &M2));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_OK, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(M4, M1);
	TEST_ASSERT_EQUAL_MATRIX(M5, M2);
}


TEST_GROUP(group_matrix_sparseSandwitch_badMats);


TEST_SETUP(group_matrix_sparseSandwitch_badMats)
{
	/* These matrix sizes are correct, but will be changed in tests */
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M2, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M2.cols));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&tmp, M1.rows, M2.cols));
	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sparseSandwitch_badMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&tmp);
}


TEST(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badInputMats)
{
	M2.rows--;
	M2.cols--;

	/* We want tmp to have right size */
	tmp.cols--;

	/* No matrix is transposed */
	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	/* First matrix is transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	/* Second matrix is transposed */
	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	/* First and second transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));
}


TEST(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badResMat)
{
	/* Incorrect rows number */
	M3.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	/* Incorrect cols number */
	M3.rows++;
	M3.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));
}


TEST(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badTmpMat)
{
	/* Incorrect rows number */
	tmp.rows--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	/* Incorrect cols number */
	tmp.rows++;
	tmp.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));
}


/* This test checks if result matrix and temporary are changing when function fails */
TEST(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_failureRetain)
{
	M2.rows--;
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M3));
	TEST_ASSERT_EQUAL_INT(MAT_BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &tmp));

	TEST_ASSERT_EQUAL_INT(MAT_SANDWITCH_FAIL, matrix_sparseSandwitch(&M1, &M2, &M3, &tmp));

	TEST_ASSERT_EQUAL_MATRIX(M4, M3);
	TEST_ASSERT_EQUAL_MATRIX(M5, tmp);
}


TEST_GROUP_RUNNER(group_matrix_sparseSandwitch)
{
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_std);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_firstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_secondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_firstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_resultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_stdMat, matrix_sparseSandwitch_allMatTrp);

	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsStd);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsFirstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsResultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_bigMatsAllMatTrp);

	RUN_TEST_CASE(group_matrix_sparseSandwitch_bigMat, matrix_sparseSandwitch_sourceRetain);

	RUN_TEST_CASE(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badInputMats);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badResMat);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_badTmpMat);
	RUN_TEST_CASE(group_matrix_sparseSandwitch_badMats, matrix_sparseSandwitch_failureRetain);
}
