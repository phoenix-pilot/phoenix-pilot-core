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

static matrix_t M1, M2, M3, Exp;


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

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_firstMatTrp)
{
	/* Matrix is changed in such way that logically it is the same but .transposed is true */
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_secondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_firstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_resultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_stdMat, matrix_prod_allMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
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
}


TEST_TEAR_DOWN(group_matrix_prod_bigMat)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
	matrix_bufFree(&Exp);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsStd)
{
	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsFirstAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndFirstMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsResultAndSecondMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


TEST(group_matrix_prod_bigMat, matrix_prod_bigMatsAllMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_transposeSwap(&M3));

	TEST_ASSERT_EQUAL_INT(PRODUCT_OK, matrix_prod(&M1, &M2, &M3));

	TEST_ASSERT_EQUAL_UINT(Exp.rows, M3.rows);
	TEST_ASSERT_EQUAL_UINT(Exp.cols, M3.cols);
	TEST_ASSERT_EQUAL_UINT(Exp.transposed, M3.transposed);
	TEST_ASSERT_EQUAL_FLOAT_ARRAY(Exp.data, M3.data, M3.rows * M3.cols);
}


/* FIXME: Add tests for incorrect matrix sizes to multiply */


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
}
