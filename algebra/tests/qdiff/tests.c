/*
 * Phoenix-Pilot
 *
 * Unit tests for quaternions differentiation library
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <unity_fixture.h>

#include <qdiff.h>

#include <string.h>

#include "../tools.h"
#include "buffs.h"

#define ROWS 4
#define COLS 4

static matrix_t M, Expected;


/* ##############################################################################
 * -----------------------        qvdiff_qpDiffQ tests       --------------------
 * ############################################################################## */

TEST_GROUP(group_qvdiff_qpDiffQ);


TEST_SETUP(group_qvdiff_qpDiffQ)
{
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M, ROWS, COLS, matInitBuff, BUFFILL_WRITE_ALL));

	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, ROWS, COLS));
}


TEST_TEAR_DOWN(group_qvdiff_qpDiffQ)
{
	matrix_bufFree(&M);
	matrix_bufFree(&Expected);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_versors)
{
	/* Testing derivative d(qp) / d(q) where p = 1 */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QA, &M));
	matrix_diag(&Expected);
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = i */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QI, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_QIpDiffQI, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = j */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QJ, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_QJpDiffQJ, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = k */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QK, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_QKpDiffQK, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_trivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&A, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_ApDiffA, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_nontrivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_BpDiffB, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_resTrp)
{
	matrix_trp(&M);

	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK, algebraTests_buffFill(&Expected, buffs_BpDiffB, ROWS * COLS));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_wrongOutputMatrixSize)
{
	matrix_bufFree(&M);

	/* Too small matrix */
	matrix_bufAlloc(&M, ROWS - 1, COLS - 1);
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffQ(&A, &M));

	matrix_bufFree(&M);

	/* Too big matrix */
	matrix_bufAlloc(&M, ROWS + 1, COLS + 1);
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffQ(&A, &M));
}


TEST_GROUP_RUNNER(group_qvdiff_qpDiffQ)
{
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_versors);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_trivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_nontrivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_resTrp);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_wrongOutputMatrixSize);
}
