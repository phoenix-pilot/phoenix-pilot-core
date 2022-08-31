/*
 * Phoenix-Pilot
 *
 * Unit tests for tools library
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


static matrix_t M1, M2;


/* ##############################################################################
 * ------------------        algebraTests_realTrp tests       -------------------
 * ############################################################################## */


TEST_GROUP(group_algebraTests_realTrp);


TEST_SETUP(group_algebraTests_realTrp)
{
	M1.data = NULL;
	M2.data = NULL;
	M1.transposed = 0;
	M2.transposed = 0;
}


TEST_TEAR_DOWN(group_algebraTests_realTrp)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	M1.transposed = 0;
	M2.transposed = 0;
}


TEST(group_algebraTests_realTrp, algebraTests_realTrp_squareMat)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));

	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);

	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_dataTrpCheck(&M1, &M2));
}


TEST(group_algebraTests_realTrp, algebraTests_realTrp_squareMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsA, buffs_colsA, buffs_A, buffs_colsA * buffs_rowsA));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));

	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);
	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_dataTrpCheck(&M1, &M2));
}


TEST(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMat)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsF, buffs_colsF, buffs_F, buffs_colsF * buffs_rowsF));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));

	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);
	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_dataTrpCheck(&M1, &M2));
}


TEST(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMatTrp)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsF, buffs_colsF, buffs_F, buffs_colsF * buffs_rowsF));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));

	TEST_ASSERT_EQUAL_UINT(M2.transposed, M1.transposed);
	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_dataTrpCheck(&M1, &M2));
}


TEST_GROUP_RUNNER(group_algebraTests_realTrp)
{
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_squareMat);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_squareMatTrp);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMat);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMatTrp);
}
