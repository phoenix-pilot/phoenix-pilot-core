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

#define ROWS_QUAT_DIFF 4
#define ROWS_VEC_DIFF  3

#define COLS_QUAT_DIFF 4
#define COLS_VEC_DIFF  3

static matrix_t M, Expected;


/* ##############################################################################
 * -----------------------        qvdiff_qpDiffQ tests       --------------------
 * ############################################################################## */

TEST_GROUP(group_qvdiff_qpDiffQ);


TEST_SETUP(group_qvdiff_qpDiffQ)
{
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK,
		algebraTests_createAndFill(&M, ROWS_QUAT_DIFF, COLS_QUAT_DIFF, matInitBuff, BUFFILL_WRITE_ALL));

	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, ROWS_QUAT_DIFF, COLS_QUAT_DIFF));
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

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = i */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QI, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QIpDiffQI, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = j */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QJ, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QJpDiffQJ, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(q) where p = k */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&QK, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QKpDiffQK, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_trivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&A, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_ApDiffA, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_nontrivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BpDiffB, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_resTrp)
{
	matrix_trp(&M);

	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffQ(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BpDiffB, ROWS_QUAT_DIFF * COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_wrongOutputMatrixSize)
{
	float buff[(ROWS_QUAT_DIFF + 1) * (COLS_QUAT_DIFF + 1)];
	matrix_t mat = { .data = buff, .transposed = 0 };

	/* Too small matrix */
	mat.rows = ROWS_QUAT_DIFF - 1;
	mat.cols = COLS_QUAT_DIFF - 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffQ(&A, &mat));

	/* Too big matrix */
	mat.rows = ROWS_QUAT_DIFF + 1;
	mat.cols = COLS_QUAT_DIFF + 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffQ(&A, &mat));
}


TEST_GROUP_RUNNER(group_qvdiff_qpDiffQ)
{
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_versors);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_trivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_nontrivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_resTrp);
	RUN_TEST_CASE(group_qvdiff_qpDiffQ, qvdiff_qpDiffQ_wrongOutputMatrixSize);
}


/* ##############################################################################
 * -----------------------        qvdiff_qpDiffP tests       --------------------
 * ############################################################################## */


TEST_GROUP(group_qvdiff_qpDiffP);


TEST_SETUP(group_qvdiff_qpDiffP)
{
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M, ROWS_QUAT_DIFF, COLS_VEC_DIFF));

	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, ROWS_QUAT_DIFF, COLS_VEC_DIFF));
}


TEST_TEAR_DOWN(group_qvdiff_qpDiffP)
{
	matrix_bufFree(&M);
	matrix_bufFree(&Expected);
}


TEST(group_qvdiff_qpDiffP, qvdiff_qpDiffP_versors)
{
	/* Testing derivative d(qp) / d(p) where q = 1 and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&QA, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QAvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(p) where q = i and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&QI, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QIvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(p) where q = j and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&QJ, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QJvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(qp) / d(p) where q = k and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&QK, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QKvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffP, qvdiff_qpDiffP_trivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&A, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_AvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffP, qvdiff_qpDiffP_nontrivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffP, qvdiff_qpDiffP_resTrp)
{
	algebraTests_transposeSwap(&M);

	TEST_ASSERT_EQUAL(0, qvdiff_qpDiffP(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BvDiffV, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qpDiffP, qvdiff_qpDiffP_wrongOutputMatrixSize)
{
	float buff[(ROWS_QUAT_DIFF + 1) * (COLS_VEC_DIFF + 1)];
	matrix_t mat = { .data = buff, .transposed = 0 };

	/* Too small matrix */
	mat.rows = ROWS_QUAT_DIFF - 1;
	mat.cols = COLS_VEC_DIFF - 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffP(&A, &mat));

	/* Too big matrix */
	mat.rows = ROWS_QUAT_DIFF + 1;
	mat.cols = COLS_VEC_DIFF + 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qpDiffP(&A, &mat));
}


TEST_GROUP_RUNNER(group_qvdiff_qpDiffP)
{
	RUN_TEST_CASE(group_qvdiff_qpDiffP, qvdiff_qpDiffP_versors);
	RUN_TEST_CASE(group_qvdiff_qpDiffP, qvdiff_qpDiffP_trivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffP, qvdiff_qpDiffP_nontrivial);
	RUN_TEST_CASE(group_qvdiff_qpDiffP, qvdiff_qpDiffP_resTrp);
	RUN_TEST_CASE(group_qvdiff_qpDiffP, qvdiff_qpDiffP_wrongOutputMatrixSize);
}


/* ##############################################################################
 * -----------------------        qvdiff_qvqDiffV tests       --------------------
 * ############################################################################## */


TEST_GROUP(group_qvdiff_qvqDiffV);


TEST_SETUP(group_qvdiff_qvqDiffV)
{
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M, ROWS_VEC_DIFF, COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, ROWS_VEC_DIFF, COLS_VEC_DIFF));
}


TEST_TEAR_DOWN(group_qvdiff_qvqDiffV)
{
	matrix_bufFree(&M);
	matrix_bufFree(&Expected);
}


TEST(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_versors)
{
	/* Testing derivative d(q*v*cjg(q)) / d(v) where q = 1 and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&QA, &M));
	matrix_diag(&Expected);

	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(v) where q = i and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&QI, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QIvQIDiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(v) where q = j and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&QJ, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QJvQJDiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));

	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(v) where q = k and p is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&QK, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QKvQKDiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_trivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&A, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_AvADiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_nontrivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BvBDiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_resTrp)
{
	matrix_trp(&M);

	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffV(&B, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_BvBDiffV, ROWS_VEC_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_wrongOutputMatrixSize)
{
	float buff[(ROWS_VEC_DIFF + 1) * (COLS_VEC_DIFF + 1)];
	matrix_t mat = { .data = buff, .transposed = 0 };

	/* Too small matrix */
	mat.rows = ROWS_VEC_DIFF - 1;
	mat.cols = COLS_VEC_DIFF - 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qvqDiffV(&A, &mat));

	/* Too big matrix */
	mat.rows = ROWS_VEC_DIFF + 1;
	mat.cols = COLS_VEC_DIFF + 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qvqDiffV(&A, &mat));
}


TEST_GROUP_RUNNER(group_qvdiff_qvqDiffV)
{
	RUN_TEST_CASE(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_versors);
	RUN_TEST_CASE(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_trivial);
	RUN_TEST_CASE(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_nontrivial);
	RUN_TEST_CASE(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_resTrp);
	RUN_TEST_CASE(group_qvdiff_qvqDiffV, qvdiff_qvqDiffV_wrongOutputMatrixSize);
}


/* ##############################################################################
 * -----------------------        qvdiff_qvqDiffQ tests       --------------------
 * ############################################################################## */


TEST_GROUP(group_qvdiff_qvqDiffQ);


TEST_SETUP(group_qvdiff_qvqDiffQ)
{
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&M, ROWS_VEC_DIFF, COLS_QUAT_DIFF));
	TEST_ASSERT_EQUAL(MAT_BUF_ALLOC_OK, matrix_bufAlloc(&Expected, ROWS_VEC_DIFF, COLS_QUAT_DIFF));
}


TEST_TEAR_DOWN(group_qvdiff_qvqDiffQ)
{
	matrix_bufFree(&M);
	matrix_bufFree(&Expected);
}


TEST(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_quaternionVersors)
{
	vec_t v = { .x = 1, .y = 1, .z = 1 };

	/* Testing derivative d(q*v*cjg(q)) / d(q) where q = 1 and v is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&QA, &v, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QAvQADiffQA, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(q) where q = i and v is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&QI, &v, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QIvQIDiffQI, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(q) where q = j and v is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&QJ, &v, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QJvQJDiffQJ, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&M, matInitBuff, BUFFILL_WRITE_ALL));

	/* Testing derivative d(q*v*cjg(q)) / d(q) where q = k and v is pure quaternion */
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&QK, &v, &M));
	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_QKvQKDiffQK, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_trivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&A, &v1, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_Av1ADiffA, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_nontrivial)
{
	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&B, &v2, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_Bv2BDiffB, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_resTrp)
{
	algebraTests_transposeSwap(&M);

	TEST_ASSERT_EQUAL(0, qvdiff_qvqDiffQ(&B, &v2, &M));

	TEST_ASSERT_EQUAL_INT(MAT_BUFFILL_OK,
		algebraTests_buffFill(&Expected, buffs_Bv2BDiffB, ROWS_QUAT_DIFF * COLS_VEC_DIFF));
	TEST_ASSERT_EQUAL_MATRIX(Expected, M);
}


TEST(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_wrongOutputMatrixSize)
{
	float buff[(ROWS_VEC_DIFF + 1) * (COLS_QUAT_DIFF + 1)];
	matrix_t mat = { .data = buff, .transposed = 0 };

	/* Too small matrix */
	mat.rows = ROWS_VEC_DIFF - 1;
	mat.cols = COLS_QUAT_DIFF - 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qvqDiffQ(&B, &v2, &mat));

	/* Too big matrix */
	mat.rows = ROWS_VEC_DIFF + 1;
	mat.cols = COLS_QUAT_DIFF + 1;
	TEST_ASSERT_NOT_EQUAL(0, qvdiff_qvqDiffQ(&B, &v2, &mat));
}


TEST_GROUP_RUNNER(group_qvdiff_qvqDiffQ)
{
	RUN_TEST_CASE(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_quaternionVersors);
	RUN_TEST_CASE(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_trivial);
	RUN_TEST_CASE(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_nontrivial);
	RUN_TEST_CASE(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_resTrp);
	RUN_TEST_CASE(group_qvdiff_qvqDiffQ, qvdiff_qvqDiffQ_wrongOutputMatrixSize);
}
