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

#include "tools.h"
#include "buffs.h"


/* Have to be bigger than 1 */
#define SQUARE_MAT_SIZE 7


static matrix_t M1, M2, M3, M4, M5, Exp;


/* ##############################################################################
 * -----------------------        matrix_add tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_add_stdMat);


TEST_SETUP(group_matrix_add_stdMat)
{
	int row, col;

	/* M1 = A */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));

	/* M2 = B */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsB, buffs_colsB, buffs_B, buffs_colsB * buffs_rowsB));

	/* Exp = A + B */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&Exp, M1.rows, M1.cols));

	/* Calculating expected result */
	for (row = 0; row < matrix_rowsGet(&Exp); row++) {
		for (col = 0; col < matrix_colsGet(&Exp); col++) {
			*matrix_at(&Exp, row, col) = *matrix_at(&M1, row, col) + *matrix_at(&M2, row, col);
		}
	}

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));
}


TEST_TEAR_DOWN(group_matrix_add_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_add_stdMat, matrix_add_std)
{
	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_stdMat, matrix_add_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST_GROUP(group_matrix_add_bigMat);


TEST_SETUP(group_matrix_add_bigMat)
{
	int row, col;

	/* M1 = E */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	/* M2 = I */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsI, buffs_colsI, buffs_I, buffs_colsI * buffs_rowsI));

	/* Exp = E + I */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&Exp, M1.rows, M1.cols));

	/* Calculating expected result */
	for (row = 0; row < matrix_rowsGet(&Exp); row++) {
		for (col = 0; col < matrix_colsGet(&Exp); col++) {
			*matrix_at(&Exp, row, col) = *matrix_at(&M1, row, col) + *matrix_at(&M2, row, col);
		}
	}

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));

	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_add_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_add_bigMat, matrix_add_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	/* Result matrix have transposition flag equal to first matrix */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&Exp));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_add_bigMat, matrix_add_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &M2));

	TEST_ASSERT_EQUAL_INT(ADD_OK, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M4, M1);
	TEST_ASSERT_EQUAL_MATRIX(M5, M2);
}


TEST_GROUP(group_matrix_add_badMats);


TEST_SETUP(group_matrix_add_badMats)
{
	/* These matrix sizes are correct, but will be changed in tests */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M2, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M2.cols));
	M4.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_add_badMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
}


TEST(group_matrix_add_badMats, matrix_add_badInputMats)
{
	M2.rows--;
	M2.cols--;

	/* No matrix is transposed */
	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	/* First matrix is transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	/* Second matrix is transposed */
	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	/* First and second transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));
}


TEST(group_matrix_add_badMats, matrix_add_badResMat)
{
	/* Incorrect rows number */
	M3.rows--;

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	/* Incorrect cols number */
	M3.rows++;
	M3.cols--;

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));
}


/* This test checks if result matrix is changing when function fails */
TEST(group_matrix_add_badMats, matrix_add_failureRetain)
{
	M2.rows--;
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M3));

	TEST_ASSERT_EQUAL_INT(ADD_FAIL, matrix_add(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M4, M3);
}


TEST_GROUP_RUNNER(group_matrix_add)
{
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_std);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_firstMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_secondMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_firstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_resultMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_resultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_resultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_add_stdMat, matrix_add_allMatTrp);

	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsStd);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsFirstMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsSecondMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsFirstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsResultMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsResultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsResultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_bigMatsAllMatTrp);

	RUN_TEST_CASE(group_matrix_add_bigMat, matrix_add_sourceRetain);

	RUN_TEST_CASE(group_matrix_add_badMats, matrix_add_badInputMats);
	RUN_TEST_CASE(group_matrix_add_badMats, matrix_add_badResMat);
	RUN_TEST_CASE(group_matrix_add_badMats, matrix_add_failureRetain);
}
