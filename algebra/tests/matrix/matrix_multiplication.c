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


/* Must be bigger than 1 */
#define SQUARE_MAT_SIZE 4


static matrix_t M1, M2, M3, M4, M5, Exp;


/* ##############################################################################
 * ----------------------        matrix_prod tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_prod_stdMat);


TEST_SETUP(group_matrix_prod_stdMat)
{
	/* M1 = C */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsC, buffs_colsC, buffs_C, buffs_colsC * buffs_rowsC));

	/* M2 = D */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsD, buffs_colsD, buffs_D, buffs_colsD * buffs_rowsD));

	/* Exp = C * D */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&Exp, buffs_rowsCtimesD, buffs_colsCtimesD, buffs_CtimesD, buffs_colsCtimesD * buffs_rowsCtimesD));

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));
}


TEST_TEAR_DOWN(group_matrix_prod_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_prod_stdMat, matrix_prod_std)
{
	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_stdMat, matrix_prod_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST_GROUP(group_matrix_prod_bigMat);


TEST_SETUP(group_matrix_prod_bigMat)
{
	/* M1 = E */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	/* M2 = F */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsF, buffs_colsF, buffs_F, buffs_colsF * buffs_rowsF));

	/* Exp = E * F */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&Exp, buffs_rowsEtimesF, buffs_colsEtimesF, buffs_EtimesF, buffs_colsEtimesF * buffs_rowsEtimesF));

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));

	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_prod_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_prod_bigMat, matrix_prod_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M1, M4);
	TEST_ASSERT_EQUAL_MATRIX(M2, M5);
}


TEST_GROUP(group_matrix_prod_badMats);


TEST_SETUP(group_matrix_prod_badMats)
{
	/* These matrix sizes are correct, but will be changed in tests */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M2, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M2.cols));
	M4.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_prod_badMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
}


TEST(group_matrix_prod_badMats, matrix_prod_badInputMats)
{
	M2.rows--;
	M2.cols--;

	/* We want M3 to have right size */
	M3.cols--;

	/* No matrix is transposed */
	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	/* First matrix is transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	/* Second matrix is transposed */
	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	/* First and second transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));
}


TEST(group_matrix_prod_badMats, matrix_prod_badResMat)
{
	/* Incorrect rows number */
	M3.rows--;

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	/* Incorrect cols number */
	M3.rows++;
	M3.cols--;

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));
}


/* This test checks if result matrix is changing when function fails */
TEST(group_matrix_prod_badMats, matrix_prod_failureRetain)
{
	M2.rows--;
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M4, M3);
}


TEST_GROUP_RUNNER(group_matrix_prod)
{
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_std);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_firstMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_secondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_firstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_resultMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_resultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_resultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_stdMat, matrix_prod_allMatTrp);

	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsStd);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsSecondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsResultMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_bigMatsAllMatTrp);

	RUN_TEST_CASE(group_matrix_prod_bigMat, matrix_prod_sourceRetain);

	RUN_TEST_CASE(group_matrix_prod_badMats, matrix_prod_badInputMats);
	RUN_TEST_CASE(group_matrix_prod_badMats, matrix_prod_badResMat);
	RUN_TEST_CASE(group_matrix_prod_badMats, matrix_prod_failureRetain);
}


/* ##############################################################################
 * -------------------        matrix_sparseProd tests       ---------------------
 * ############################################################################## */


TEST_GROUP(group_matrix_sparseProd_stdMat);


TEST_SETUP(group_matrix_sparseProd_stdMat)
{
	/* M1 = C */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsC, buffs_colsC, buffs_C, buffs_colsC * buffs_rowsC));

	/* M2 = D */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsD, buffs_colsD, buffs_D, buffs_colsD * buffs_rowsD));

	/* Exp = C * D */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&Exp, buffs_rowsCtimesD, buffs_colsCtimesD, buffs_CtimesD, buffs_colsCtimesD * buffs_rowsCtimesD));

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));
}


TEST_TEAR_DOWN(group_matrix_sparseProd_stdMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_std)
{
	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_stdMat, matrix_sparseProd_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST_GROUP(group_matrix_sparseProd_bigMat);


TEST_SETUP(group_matrix_sparseProd_bigMat)
{
	/* M1 = E */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsE, buffs_colsE, buffs_E, buffs_colsE * buffs_rowsE));

	/* M2 = F */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M2, buffs_rowsF, buffs_colsF, buffs_F, buffs_colsF * buffs_rowsF));

	/* Exp = E * F */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&Exp, buffs_rowsEtimesF, buffs_colsEtimesF, buffs_EtimesF, buffs_colsEtimesF * buffs_rowsEtimesF));

	/* Allocating matrix for results */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, Exp.rows, Exp.cols));

	M4.data = NULL;
	M5.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sparseProd_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
	matrix_bufFree(&M5);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(Exp, M3);
}


/* This tests checks if function changes source matrices after success */
TEST(group_matrix_sparseProd_bigMat, matrix_sparseProd_sourceRetain)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M5, &M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M4, M1);
	TEST_ASSERT_EQUAL_MATRIX(M5, M2);
}


TEST_GROUP(group_matrix_sparseProd_badMats);


TEST_SETUP(group_matrix_sparseProd_badMats)
{
	/* These matrix sizes are correct, but will be changed in tests */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M1, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M2, SQUARE_MAT_SIZE, SQUARE_MAT_SIZE));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, matrix_bufAlloc(&M3, M1.rows, M2.cols));
	M4.data = NULL;
}


TEST_TEAR_DOWN(group_matrix_sparseProd_badMats)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&M4);
}


TEST(group_matrix_sparseProd_badMats, matrix_sparseProd_badInputMats)
{
	M2.rows--;
	M2.cols--;

	/* We want M3 to have right size */
	M3.cols--;

	/* No matrix is transposed */
	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	/* First matrix is transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	/* Second matrix is transposed */
	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	/* First and second transposed */
	matrix_trp(&M1);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));
}


TEST(group_matrix_sparseProd_badMats, matrix_sparseProd_badResMat)
{
	/* Incorrect rows number */
	M3.rows--;

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	/* Incorrect cols number */
	M3.rows++;
	M3.cols--;

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	matrix_trp(&M3);

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));
}


/* This test checks if result matrix is changing when function fails */
TEST(group_matrix_sparseProd_badMats, matrix_sparseProd_failureRetain)
{
	M2.rows--;
	M2.cols--;

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M4, &M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_FAIL, matrix_sparseProd(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_MATRIX(M3, M4);
}


TEST_GROUP_RUNNER(group_matrix_sparseProd)
{
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_std);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_firstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_secondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_firstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_resultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_stdMat, matrix_sparseProd_allMatTrp);

	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsStd);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsFirstAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultAndFirstMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsResultAndSecondMatTrp);
	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_bigMatsAllMatTrp);

	RUN_TEST_CASE(group_matrix_sparseProd_bigMat, matrix_sparseProd_sourceRetain);

	RUN_TEST_CASE(group_matrix_sparseProd_badMats, matrix_sparseProd_badInputMats);
	RUN_TEST_CASE(group_matrix_sparseProd_badMats, matrix_sparseProd_badResMat);
	RUN_TEST_CASE(group_matrix_sparseProd_badMats, matrix_sparseProd_failureRetain);
}
