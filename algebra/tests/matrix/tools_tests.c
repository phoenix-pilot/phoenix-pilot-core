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


static matrix_t M1, M2, M3;


/* ##############################################################################
 * ------------------        algebraTests_realTrp tests       -------------------
 * ############################################################################## */


TEST_GROUP(group_algebraTests_realTrp);


TEST_SETUP(group_algebraTests_realTrp)
{
	M1.data = NULL;
	M2.data = NULL;
}


TEST_TEAR_DOWN(group_algebraTests_realTrp)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
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


TEST(group_algebraTests_realTrp, algebraTests_realTrp_trpTwoTimes)
{
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK,
		algebraTests_createAndFill(&M1, buffs_rowsF, buffs_colsF, buffs_F, buffs_colsF * buffs_rowsF));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_matrixCopy(&M2, &M1));

	matrix_trp(&M1);
	matrix_trp(&M2);

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_realTrp(&M1));

	TEST_ASSERT_EQUAL_MATRIX(M1, M2);
}


TEST_GROUP_RUNNER(group_algebraTests_realTrp)
{
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_squareMat);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_squareMatTrp);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMat);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_notSquareMatTrp);
	RUN_TEST_CASE(group_algebraTests_realTrp, algebraTests_realTrp_trpTwoTimes);
}


/* ##############################################################################
 * -------------        algebraTests_submatCheck tests       ---------------
 * ############################################################################## */


TEST_GROUP(group_algebraTests_submatCheck);


TEST_SETUP(group_algebraTests_submatCheck)
{
	int rowsM1 = 3, colsM1 = 3;
	float dataM1[] = {
		1.0, 2.0, 3.0,
		4.0, 0.0, 0.0,
		7.0, 0.0, 0.0
	};

	int rowsM2 = 2, colsM2 = 2;
	float dataM2[] = {
		5.0, 6.0,
		8.0, 9.0
	};

	int rowsM3 = 3, colsM3 = 3;
	float dataM3[] = {
		1.0, 2.0, 3.0,
		4.0, 5.0, 6.0,
		7.0, 8.0, 9.0
	};

	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_createAndFill(&M1, rowsM1, colsM1, dataM1, rowsM1 * colsM1));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_createAndFill(&M2, rowsM2, colsM2, dataM2, rowsM2 * colsM2));
	TEST_ASSERT_EQUAL_INT(BUF_ALLOC_OK, algebraTests_createAndFill(&M3, rowsM3, colsM3, dataM3, rowsM3 * colsM3));
}


TEST_TEAR_DOWN(group_algebraTests_submatCheck)
{
	matrix_bufFree(&M1);
	matrix_bufFree(&M2);
	matrix_bufFree(&M3);
}


TEST(group_algebraTests_submatCheck, algebraTests_submatCheck_std)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK, algebraTests_submatCheck(&M1, 1, 1, &M2, &M3));

	(*matrix_at(&M3, M3.rows / 2, M3.cols / 2))++;

	TEST_ASSERT_EQUAL_INT(CHECK_FAIL, algebraTests_submatCheck(&M1, 1, 1, &M2, &M3));
}


TEST_GROUP_RUNNER(group_algebraTests_submatCheck)
{
	RUN_TEST_CASE(group_algebraTests_submatCheck, algebraTests_submatCheck_std);
}
