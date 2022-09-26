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

#include <quat.h>

#include <math.h>

#include "../buffs.h"
#include "../tools.h"


/* ##############################################################################
 * ------------------------        quat_cmp tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_cmp);


TEST_SETUP(group_quat_cmp)
{
}


TEST_TEAR_DOWN(group_quat_cmp)
{
}


TEST(group_quat_cmp, quat_cmp_stdPass)
{
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &Q2));
}


TEST(group_quat_cmp, quat_cmp_different)
{
	quat_t A, I, J, K, Q;

	A = I = J = K = Q = Q2;

	A.a++;
	I.i++;
	J.j++;
	K.k++;

	Q.a++;
	Q.i++;
	Q.j++;
	Q.k++;

	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &A));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &I));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &J));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &K));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &Q));
}


TEST_GROUP_RUNNER(group_quat_cmp)
{
	RUN_TEST_CASE(group_quat_cmp, quat_cmp_stdPass);
	RUN_TEST_CASE(group_quat_cmp, quat_cmp_different);
}


/* ##############################################################################
 * ---------------------        quat_idenWrite tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_idenWrite);


TEST_SETUP(group_quat_idenWrite)
{
}


TEST_TEAR_DOWN(group_quat_idenWrite)
{
}


TEST(group_quat_idenWrite, quat_idenWrite_std)
{
	/* Initialising quaternion with non zero and non one values */
	quat_t A = Q1;

	quat_idenWrite(&A);

	TEST_ASSERT_EQUAL_QUAT(QA, A);
}


TEST_GROUP_RUNNER(group_quat_idenWrite)
{
	RUN_TEST_CASE(group_quat_idenWrite, quat_idenWrite_std);
}


/* ##############################################################################
 * ----------------------        quat_piWrite tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_piWrite);


TEST_SETUP(group_quat_piWrite)
{
}


TEST_TEAR_DOWN(group_quat_piWrite)
{
}


TEST(group_quat_piWrite, quat_piWrite_std)
{
	/* Initialising quaternion with non zero and non one values */
	quat_t A = Q1;

	quat_piWrite(&A);

	TEST_ASSERT_EQUAL_QUAT(QI, A);
}


TEST_GROUP_RUNNER(group_quat_piWrite)
{
	RUN_TEST_CASE(group_quat_piWrite, quat_piWrite_std);
}


/* ##############################################################################
 * ------------------------        quat_add tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_add);


TEST_SETUP(group_quat_add)
{
}


TEST_TEAR_DOWN(group_quat_add)
{
}


TEST(group_quat_add, quat_add_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t expected = { .a = A.a + B.a, .i = A.i + B.i, .j = A.j + B.j, .k = A.k + B.k };

	quat_add(&A, &B);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_add, quat_add_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t expected = { .a = A.a + B.a, .i = A.i + B.i, .j = A.j + B.j, .k = A.k + B.k };

	quat_add(&A, &B);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST_GROUP_RUNNER(group_quat_add)
{
	RUN_TEST_CASE(group_quat_add, quat_add_std);
	RUN_TEST_CASE(group_quat_add, quat_add_biggerValues);
}


/* ##############################################################################
 * ------------------------        quat_sum tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_sum);


TEST_SETUP(group_quat_sum)
{
}


TEST_TEAR_DOWN(group_quat_sum)
{
}


TEST(group_quat_sum, quat_sum_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t C;
	quat_t expected = { .a = A.a + B.a, .i = A.i + B.i, .j = A.j + B.j, .k = A.k + B.k };

	quat_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(expected, C);
}


TEST(group_quat_sum, quat_sum_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;
	quat_t expected = { .a = A.a + B.a, .i = A.i + B.i, .j = A.j + B.j, .k = A.k + B.k };

	quat_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(expected, C);
}


TEST_GROUP_RUNNER(group_quat_sum)
{
	RUN_TEST_CASE(group_quat_sum, quat_sum_std);
	RUN_TEST_CASE(group_quat_sum, quat_sum_biggerValues);
}


/* ##############################################################################
 * ------------------------        quat_sub tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_sub);


TEST_SETUP(group_quat_sub)
{
}


TEST_TEAR_DOWN(group_quat_sub)
{
}


TEST(group_quat_sub, quat_sub_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t expected = { .a = A.a - B.a, .i = A.i - B.i, .j = A.j - B.j, .k = A.k - B.k };

	quat_sub(&A, &B);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_sub, quat_sub_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t expected = { .a = A.a - B.a, .i = A.i - B.i, .j = A.j - B.j, .k = A.k - B.k };

	quat_sub(&A, &B);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST_GROUP_RUNNER(group_quat_sub)
{
	RUN_TEST_CASE(group_quat_sub, quat_sub_std);
	RUN_TEST_CASE(group_quat_sub, quat_sub_biggerValues);
}


/* ##############################################################################
 * ------------------------        quat_dif tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_dif);


TEST_SETUP(group_quat_dif)
{
}


TEST_TEAR_DOWN(group_quat_dif)
{
}


TEST(group_quat_dif, quat_dif_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t C;
	quat_t expected = { .a = A.a - B.a, .i = A.i - B.i, .j = A.j - B.j, .k = A.k - B.k };

	quat_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(expected, C);
}


TEST(group_quat_dif, quat_dif_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;
	quat_t expected = { .a = A.a - B.a, .i = A.i - B.i, .j = A.j - B.j, .k = A.k - B.k };

	quat_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(expected, C);
}


TEST_GROUP_RUNNER(group_quat_dif)
{
	RUN_TEST_CASE(group_quat_dif, quat_dif_std);
	RUN_TEST_CASE(group_quat_dif, quat_dif_biggerValues);
}


/* ##############################################################################
 * ----------------------        quat_mlt tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_mlt);


TEST_SETUP(group_quat_mlt)
{
}


TEST_TEAR_DOWN(group_quat_mlt)
{
}


TEST(group_quat_mlt, quat_mlt_quatMltTable)
{
	quat_t nqA, nqI, nqJ, nqK;
	quat_t Res;

	nqA = nqI = nqJ = nqK = Q0;            /* zero all quaternions */
	nqA.a = nqI.i = nqJ.j = nqK.k = -1.0f; /* set all negative unitary quaternions */

	/* Checking if 1 * 1 is equal to 1 */
	quat_mlt(&QA, &QA, &Res);
	TEST_ASSERT_EQUAL_QUAT(QA, Res);

	/* Checking if 1 * i is equal to i */
	quat_mlt(&QA, &QI, &Res);
	TEST_ASSERT_EQUAL_QUAT(QI, Res);

	/* Checking if 1 * j is equal to j */
	quat_mlt(&QA, &QJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(QJ, Res);

	/* Checking if 1 * k is equal to k */
	quat_mlt(&QA, &QK, &Res);
	TEST_ASSERT_EQUAL_QUAT(QK, Res);

	/* Checking if i * 1 is equal to i */
	quat_mlt(&QI, &QA, &Res);
	TEST_ASSERT_EQUAL_QUAT(QI, Res);

	/* Checking if i * i is equal to -1 */
	quat_mlt(&QI, &QI, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqA, Res);

	/* Checking if i * j is equal to k */
	quat_mlt(&QI, &QJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(QK, Res);

	/* Checking if i * k is equal to -j */
	quat_mlt(&QI, &QK, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqJ, Res);

	/* Checking if j * 1 is equal to j */
	quat_mlt(&QJ, &QA, &Res);
	TEST_ASSERT_EQUAL_QUAT(QJ, Res);

	/* Checking if j * i is equal to -k */
	quat_mlt(&QJ, &QI, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqK, Res);

	/* Checking if j * j is equal to -1 */
	quat_mlt(&QJ, &QJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqA, Res);

	/* Checking if j * k is equal to i */
	quat_mlt(&QJ, &QK, &Res);
	TEST_ASSERT_EQUAL_QUAT(QI, Res);

	/* Checking if k * 1 is equal to k */
	quat_mlt(&QK, &QA, &Res);
	TEST_ASSERT_EQUAL_QUAT(QK, Res);

	/* Checking if k * i is equal to j */
	quat_mlt(&QK, &QI, &Res);
	TEST_ASSERT_EQUAL_QUAT(QJ, Res);

	/* Checking if k * j is equal to -i */
	quat_mlt(&QK, &QJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqI, Res);

	/* Checking if k * k is equal to -1 */
	quat_mlt(&QK, &QK, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqA, Res);
}


TEST(group_quat_mlt, quat_mlt_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t C;

	quat_mlt(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(Q2timesQ3, C);
}


TEST(group_quat_mlt, quat_mlt_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;

	quat_mlt(&A, &B, &C);

	TEST_ASSERT_EQUAL_QUAT(Q4timesQ5, C);
}


TEST_GROUP_RUNNER(group_quat_mlt)
{
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_quatMltTable);
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_std);
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_biggerValues);
}


/* ##############################################################################
 * -----------------------        quat_times tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_times);


TEST_SETUP(group_quat_times)
{
}


TEST_TEAR_DOWN(group_quat_times)
{
}


TEST(group_quat_times, quat_times_std)
{
	quat_t A = Q5;
	quat_t expected = {
		.a = A.a * POS_SCALAR,
		.i = A.i * POS_SCALAR,
		.j = A.j * POS_SCALAR,
		.k = A.k * POS_SCALAR
	};

	quat_times(&A, POS_SCALAR);

	TEST_ASSERT_EQUAL_QUAT(expected, A);

	A = Q5;
	expected.a = A.a * NEG_SCALAR;
	expected.i = A.i * NEG_SCALAR;
	expected.j = A.j * NEG_SCALAR;
	expected.k = A.k * NEG_SCALAR;

	quat_times(&A, NEG_SCALAR);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_times, quat_times_infs)
{
	quat_t A = Q5;
	quat_t expected = {
		.a = A.a * INFINITY,
		.i = A.i * INFINITY,
		.j = A.j * INFINITY,
		.k = A.k * INFINITY
	};

	quat_times(&A, INFINITY);

	TEST_ASSERT_EQUAL_QUAT(expected, A);

	A = Q5;
	expected.a = A.a * -INFINITY;
	expected.i = A.i * -INFINITY;
	expected.j = A.j * -INFINITY;
	expected.k = A.k * -INFINITY;

	quat_times(&A, -INFINITY);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_times, quat_times_nan)
{
	quat_t A = Q5;
	quat_t expected = { .a = A.a * NAN, .i = A.i * NAN, .j = A.j * NAN, .k = A.k * NAN };

	quat_times(&A, NAN);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST_GROUP_RUNNER(group_quat_times)
{
	RUN_TEST_CASE(group_quat_times, quat_times_std);
	RUN_TEST_CASE(group_quat_times, quat_times_infs);
	RUN_TEST_CASE(group_quat_times, quat_times_nan);
}


/* ##############################################################################
 * ------------------------        quat_cjg tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_cjg);


TEST_SETUP(group_quat_cjg)
{
}


TEST_TEAR_DOWN(group_quat_cjg)
{
}


TEST(group_quat_cjg, quat_cjg_std)
{
	quat_t A = Q2;
	quat_t expected = { .a = A.a, .i = -A.i, .j = -A.j, .k = -A.k };

	quat_cjg(&A);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_cjg, quat_cjg_biggerValues)
{
	quat_t A = Q4;
	quat_t expected = { .a = A.a, .i = -A.i, .j = -A.j, .k = -A.k };

	quat_cjg(&A);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
}


TEST(group_quat_cjg, quat_cjg_zeroQuat)
{
	quat_t a = Q0;
	quat_t expected = Q0;

	quat_cjg(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST_GROUP_RUNNER(group_quat_cjg)
{
	RUN_TEST_CASE(group_quat_cjg, quat_cjg_std);
	RUN_TEST_CASE(group_quat_cjg, quat_cjg_biggerValues);
	RUN_TEST_CASE(group_quat_cjg, quat_cjg_zeroQuat);
}


/* ##############################################################################
 * ------------------------        quat_dot tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_dot);


TEST_SETUP(group_quat_dot)
{
}


TEST_TEAR_DOWN(group_quat_dot)
{
}


TEST(group_quat_dot, quat_dot_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	float expected = A.a * B.a + A.i * B.i + A.j * B.j + A.k * B.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&A, &B));
}


TEST(group_quat_dot, quat_dot_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	float expected = A.a * B.a + A.i * B.i + A.j * B.j + A.k * B.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&A, &B));
}


TEST(group_quat_dot, quat_dot_perpendicular)
{
	quat_t A = Q6;
	quat_t B = Q7;
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&A, &B));
	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&B, &A));
}


TEST(group_quat_dot, quat_dot_parallel)
{
	quat_t A = Q2;
	quat_t B = A;
	float expected;

	/* Parallel with common direction */
	quat_times(&B, POS_SCALAR);
	expected = A.a * B.a + A.i * B.i + A.j * B.j + A.k * B.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&A, &B));

	/* Parallel with opposite directions */
	B = A;
	quat_times(&B, NEG_SCALAR);
	expected = A.a * B.a + A.i * B.i + A.j * B.j + A.k * B.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&A, &B));
}


TEST_GROUP_RUNNER(group_quat_dot)
{
	RUN_TEST_CASE(group_quat_dot, quat_dot_std);
	RUN_TEST_CASE(group_quat_dot, quat_dot_biggerValues);
	RUN_TEST_CASE(group_quat_dot, quat_dot_perpendicular);
	RUN_TEST_CASE(group_quat_dot, quat_dot_parallel);
}


/* ##############################################################################
 * ---------------------        quat_sandwich tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_sandwich);


TEST_SETUP(group_quat_sandwich)
{
}


TEST_TEAR_DOWN(group_quat_sandwich)
{
}


TEST(group_quat_sandwich, quat_sandwich_std)
{
	quat_t c;

	quat_sandwich(&Q2, &Q3, &c);

	TEST_ASSERT_EQUAL_QUAT(Q2sandQ3, c);
}


TEST(group_quat_sandwich, quat_sandwich_biggerValues)
{
	quat_t c;

	quat_sandwich(&Q4, &Q5, &c);

	TEST_ASSERT_EQUAL_QUAT(Q4sandQ5, c);
}


TEST_GROUP_RUNNER(group_quat_sandwich)
{
	RUN_TEST_CASE(group_quat_sandwich, quat_sandwich_std);
	RUN_TEST_CASE(group_quat_sandwich, quat_sandwich_biggerValues);
}


/* ##############################################################################
 * ---------------------        quat_normalize tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_normalize);


TEST_SETUP(group_quat_normalize)
{
}


TEST_TEAR_DOWN(group_quat_normalize)
{
}


TEST(group_quat_normalize, quat_normalize_std)
{
	quat_t a = Q2;
	float len = sqrt(a.a * a.a + a.i * a.i + a.j * a.j + a.k * a.k);
	quat_t expected = { .a = a.a / len, .i = a.i / len, .j = a.j / len, .k = a.k / len };

	quat_normalize(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_normalize, quat_normalize_biggerValues)
{
	quat_t a = Q4;
	float len = sqrt(a.a * a.a + a.i * a.i + a.j * a.j + a.k * a.k);
	quat_t expected = { .a = a.a / len, .i = a.i / len, .j = a.j / len, .k = a.k / len };

	quat_normalize(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_normalize, quat_normalize_zeroQuat)
{
	quat_t a = Q0;
	quat_t expected = Q0;

	quat_normalize(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST_GROUP_RUNNER(group_quat_normalize)
{
	RUN_TEST_CASE(group_quat_normalize, quat_normalize_std);
	RUN_TEST_CASE(group_quat_normalize, quat_normalize_biggerValues);
	RUN_TEST_CASE(group_quat_normalize, quat_normalize_zeroQuat);
}
