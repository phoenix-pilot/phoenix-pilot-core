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


#define DELTA 1e-7


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
	quat_t expected = { .a = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f };

	quat_idenWrite(&A);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
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
	quat_t expected = { .a = 0.0f, .i = 1.0f, .j = 0.0f, .k = 0.0f };

	quat_piWrite(&A);

	TEST_ASSERT_EQUAL_QUAT(expected, A);
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
	quat_t qA, qI, qJ, qK, nqA, nqI, nqJ, nqK;
	quat_t Res, qZero = { 0 };

	qA = qI = qJ = qK = nqA = nqI = nqJ = nqK = qZero; /* zero all quaternions */
	qA.a = qI.i = qJ.j = qK.k = 1.0f;                  /* set all positive unitary quaternions */
	nqA.a = nqI.i = nqJ.j = nqK.k = -1.0f;             /* set all negative unitary quaternions */

	/* Checking if 1 * 1 is equal to 1 */
	quat_mlt(&qA, &qA, &Res);
	TEST_ASSERT_EQUAL_QUAT(qA, Res);

	/* Checking if 1 * i is equal to i */
	quat_mlt(&qA, &qI, &Res);
	TEST_ASSERT_EQUAL_QUAT(qI, Res);

	/* Checking if 1 * j is equal to j */
	quat_mlt(&qA, &qJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(qJ, Res);

	/* Checking if 1 * k is equal to k */
	quat_mlt(&qA, &qK, &Res);
	TEST_ASSERT_EQUAL_QUAT(qK, Res);

	/* Checking if i * 1 is equal to i */
	quat_mlt(&qI, &qA, &Res);
	TEST_ASSERT_EQUAL_QUAT(qI, Res);

	/* Checking if i * i is equal to -1 */
	quat_mlt(&qI, &qI, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqA, Res);

	/* Checking if i * j is equal to k */
	quat_mlt(&qI, &qJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(qK, Res);

	/* Checking if i * k is equal to -j */
	quat_mlt(&qI, &qK, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqJ, Res);

	/* Checking if j * 1 is equal to j */
	quat_mlt(&qJ, &qA, &Res);
	TEST_ASSERT_EQUAL_QUAT(qJ, Res);

	/* Checking if j * i is equal to -k */
	quat_mlt(&qJ, &qI, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqK, Res);

	/* Checking if j * j is equal to -1 */
	quat_mlt(&qJ, &qJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqA, Res);

	/* Checking if j * k is equal to i */
	quat_mlt(&qJ, &qK, &Res);
	TEST_ASSERT_EQUAL_QUAT(qI, Res);

	/* Checking if k * 1 is equal to k */
	quat_mlt(&qK, &qA, &Res);
	TEST_ASSERT_EQUAL_QUAT(qK, Res);

	/* Checking if k * i is equal to j */
	quat_mlt(&qK, &qI, &Res);
	TEST_ASSERT_EQUAL_QUAT(qJ, Res);

	/* Checking if k * j is equal to -i */
	quat_mlt(&qK, &qJ, &Res);
	TEST_ASSERT_EQUAL_QUAT(nqI, Res);

	/* Checking if k * k is equal to -1 */
	quat_mlt(&qK, &qK, &Res);
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


/* ##############################################################################
 * --------------------        quat_quat2euler tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_quat2euler);


TEST_SETUP(group_quat_quat2euler)
{
}


TEST_TEAR_DOWN(group_quat_quat2euler)
{
}


TEST(group_quat_quat2euler, quat_quat2euler_baseQuaternions)
{
	quat_t qA = { .a = 1.0, .i = 0.0, .j = 0.0, .k = 0.0 };
	quat_t qI = { .a = 0.0, .i = 1.0, .j = 0.0, .k = 0.0 };
	quat_t qJ = { .a = 0.0, .i = 0.0, .j = 1.0, .k = 0.0 };
	quat_t qK = { .a = 0.0, .i = 0.0, .j = 0.0, .k = 1.0 };

	float roll, pitch, yaw;

	/* quat(1,0,0,0) should give (yaw=0, roll=0, pitch=0) */
	quat_quat2euler(&qA, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, yaw);

	/* quat(0,1,0,0) should give (yaw=0, roll=PI, pitch=0) */
	quat_quat2euler(&qI, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, yaw);

	/* quat(0,0,1,0) should give (yaw=PI, roll=0, pitch=PI) */
	quat_quat2euler(&qJ, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, yaw);

	/* quat(0,0,0,1) should give (yaw=PI, roll=0, pitch=0) */
	quat_quat2euler(&qK, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, yaw);
}


TEST(group_quat_quat2euler, quat_quat2euler_notUnitQuat)
{
	float roll, pitch, yaw;
	quat_t a = Q3;

	quat_quat2euler(&a, &roll, &pitch, &yaw);

	TEST_ASSERT_EQUAL_FLOAT(Q3roll, roll);
	TEST_ASSERT_EQUAL_FLOAT(Q3pitch, pitch);
	TEST_ASSERT_EQUAL_FLOAT(Q3yaw, yaw);
}


TEST(group_quat_quat2euler, quat_quat2euler_notUnitBiggerValues)
{
	float roll, pitch, yaw;
	quat_t a = Q5;

	quat_quat2euler(&a, &roll, &pitch, &yaw);

	TEST_ASSERT_EQUAL_FLOAT(Q5roll, roll);
	TEST_ASSERT_EQUAL_FLOAT(Q5pitch, pitch);
	TEST_ASSERT_EQUAL_FLOAT(Q5yaw, yaw);
}


TEST(group_quat_quat2euler, quat_quat2euler_sourceRetain)
{
	float roll, pitch, yaw;
	quat_t a = Q5;

	quat_quat2euler(&a, &roll, &pitch, &yaw);

	TEST_ASSERT_EQUAL_QUAT(Q5, a);
}


TEST_GROUP_RUNNER(group_quat_quat2euler)
{
	RUN_TEST_CASE(group_quat_quat2euler, quat_quat2euler_baseQuaternions);
	RUN_TEST_CASE(group_quat_quat2euler, quat_quat2euler_notUnitQuat);
	RUN_TEST_CASE(group_quat_quat2euler, quat_quat2euler_notUnitBiggerValues);
	RUN_TEST_CASE(group_quat_quat2euler, quat_quat2euler_sourceRetain);
}


/* ##############################################################################
 * -----------------------        quat_vecRot tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_vecRot);


TEST_SETUP(group_quat_vecRot)
{
}


TEST_TEAR_DOWN(group_quat_vecRot)
{
}


TEST(group_quat_vecRot, quat_vecRot_baseQuaternions)
{
	quat_t qA, qI, qJ, qK, qZero = { 0 };
	vec_t v1 = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
	vec_t v2 = { .x = -1.0f, .y = 0.0f, .z = 0.0f };
	vec_t v, expected;

	qA = qI = qJ = qK = qZero;
	qA.a = qI.i = qJ.j = qK.k = 1.0f;

	/* Rotation using qA quaternion. Nothing should change */
	v = v1;
	expected = v1;

	quat_vecRot(&v, &qA);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qI quaternion should be equal to rotation about 180 degrees along x-axis. Nothing should change */
	v = v1;
	expected = v1;

	quat_vecRot(&v, &qI);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qJ quaternion should be equal to rotation about 180 degrees along y-axis. */
	v = v1;
	expected = v2;

	quat_vecRot(&v, &qJ);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qK quaternion should be equal to rotation about 180 degrees along z-axis. */
	v = v1;
	expected = v2;

	quat_vecRot(&v, &qK);

	TEST_ASSERT_EQUAL_VEC(expected, v);
}


TEST(group_quat_vecRot, quat_vecRot_std)
{
	vec_t v = V8;

	quat_vecRot(&v, &Q8);

	TEST_ASSERT_EQUAL_VEC(V8rotQ8, v);
}


TEST(group_quat_vecRot, quat_vecRot_biggerValues)
{
	vec_t v = V9;

	quat_vecRot(&v, &Q8);

	TEST_ASSERT_EQUAL_VEC(V9rotQ8, v);
}


TEST(group_quat_vecRot, quat_vecRot_zeroVector)
{
	vec_t v1, zeroVec = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

	v1 = zeroVec;

	quat_vecRot(&v1, &Q8);

	TEST_ASSERT_EQUAL_VEC(zeroVec, v1);
}


TEST_GROUP_RUNNER(group_quat_vecRot)
{
	RUN_TEST_CASE(group_quat_vecRot, quat_vecRot_baseQuaternions);
	RUN_TEST_CASE(group_quat_vecRot, quat_vecRot_std);
	RUN_TEST_CASE(group_quat_vecRot, quat_vecRot_biggerValues);
	RUN_TEST_CASE(group_quat_vecRot, quat_vecRot_zeroVector);
}


/* ##############################################################################
 * ------------------        quat_rotQuat tests       --------------------
 * ############################################################################## */


TEST_GROUP(group_quat_rotQuat);


TEST_SETUP(group_quat_rotQuat)
{
}


TEST_TEAR_DOWN(group_quat_rotQuat)
{
}


TEST(group_quat_rotQuat, quat_rotQuat_baseQuaternions)
{
	quat_t q, qA, qI, qJ, qK, qZero = { 0 };
	const vec_t versX = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
	const vec_t versY = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
	const vec_t versZ = { .x = 0.0f, .y = 0.0f, .z = 1.0f };

	qA = qI = qJ = qK = qZero;
	qA.a = qI.i = qJ.j = qK.k = 1.0f;

	/* Rotation about 0 angle - quaternion qA */
	quat_rotQuat(&versX, 0.0, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, qA, q);

	/* Rotation about 180 degrees along x-axis - quaternion qI */
	quat_rotQuat(&versX, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, qI, q);

	/* Rotation about 180 degrees along y-axis - quaternion qJ */
	quat_rotQuat(&versY, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, qJ, q);

	/* Rotation about 180 degrees along z-axis - quaternion qK */
	quat_rotQuat(&versZ, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, qK, q);
}


TEST(group_quat_rotQuat, quat_rotQuat_std)
{
	quat_t q, expected = Q9;

	quat_rotQuat(&V10, Angle, &q);

	TEST_ASSERT_EQUAL_QUAT(expected, q);
}


TEST(group_quat_rotQuat, quat_rotQuat_zeroVector)
{
	vec_t zeroVec = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
	quat_t q, expected;

	quat_idenWrite(&expected);

	quat_rotQuat(&zeroVec, Angle, &q);

	TEST_ASSERT_EQUAL_QUAT(expected, q);
}


TEST_GROUP_RUNNER(group_quat_rotQuat)
{
	RUN_TEST_CASE(group_quat_rotQuat, quat_rotQuat_baseQuaternions);
	RUN_TEST_CASE(group_quat_rotQuat, quat_rotQuat_std);
	RUN_TEST_CASE(group_quat_rotQuat, quat_rotQuat_zeroVector);
}


/* ##############################################################################
 * ---------------------        quat_uvec2uvec tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_uvec2uvec);


TEST_SETUP(group_quat_uvec2uvec)
{
}


TEST_TEAR_DOWN(group_quat_uvec2uvec)
{
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_std)
{
	quat_t q;
	vec_t v1 = V8;
	vec_t v2 = V8rotQ8;

	vec_normalize(&v1);
	vec_normalize(&v2);

	quat_uvec2uvec(&v1, &v2, &q);

	/* We are not comparing `q` to `Q8`, because there is infinite number of correct quaternions, which rotates `v1` to `v2` */
	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_biggerValues)
{
	quat_t q;
	vec_t v1 = V9;
	vec_t v2 = V9rotQ8;

	vec_normalize(&v1);
	vec_normalize(&v2);

	quat_uvec2uvec(&v1, &v2, &q);

	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_parallel)
{
	quat_t q, expected;
	vec_t v1 = V9, v2;

	quat_idenWrite(&expected);

	/* Making `v1` and `v2` unitary and parallel */
	vec_normalize(&v1);
	v2 = v1;

	quat_uvec2uvec(&v1, &v2, &q);

	TEST_ASSERT_EQUAL_QUAT(expected, q);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_antiparallel)
{
	quat_t q;
	vec_t v1 = V9, v2;

	/* Making `v1` and `v2` unitary and antiparallel */
	vec_normalize(&v1);
	v2 = v1;
	vec_times(&v2, -1);

	quat_uvec2uvec(&v1, &v2, &q);

	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_nearlyAntiparallel)
{
	quat_t q;
	vec_t v1 = V10, v2 = V11;

	vec_normalize(&v1);
	vec_normalize(&v2);

	quat_uvec2uvec(&v1, &v2, &q);

	/* `v1` and `v2` are nearly antiparallel, but not enough for `quat_uvec2uvec` to be perceived as antiparallel */
	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_nearlyParallel)
{
	quat_t q;
	vec_t v1 = V10, v2 = V12;

	vec_normalize(&v1);
	vec_normalize(&v2);

	quat_uvec2uvec(&v1, &v2, &q);

	/* `v1` and `v2` are nearly antiparallel, but not enough for `quat_uvec2uvec` to be perceived as antiparallel */
	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST_GROUP_RUNNER(group_quat_uvec2uvec)
{
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_std);
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_biggerValues);
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_parallel);
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_antiparallel);
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_nearlyAntiparallel);
	RUN_TEST_CASE(group_quat_uvec2uvec, quat_uvec2uvec_nearlyParallel);
}


/* ##############################################################################
 * ---------------------        quat_frameRot tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_quat_frameRot);


TEST_SETUP(group_quat_frameRot)
{
}


TEST_TEAR_DOWN(group_quat_frameRot)
{
}


TEST(group_quat_frameRot, quat_frameRot_baseRot)
{
	const vec_t versX = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
	const vec_t versY = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
	const vec_t versZ = { .x = 0.0f, .y = 0.0f, .z = 1.0f };
	const vec_t negVersY = { .x = 0.0f, .y = -1.0f, .z = 0.0f };
	quat_t q, expected, helpQ;

	/* Rotating frame of reference 90 degrees along x-axis */
	quat_rotQuat(&versX, M_PI_2, &expected);
	quat_rotQuat(&versX, M_PI_4, &helpQ);

	quat_frameRot(&versX, &versY, &versX, &versZ, &q, &helpQ);

	TEST_ASSERT_EQUAL_QUAT(expected, q);

	/* Rotating frame of reference 90 degrees along y-axis */
	quat_rotQuat(&versY, M_PI_2, &expected);
	quat_rotQuat(&versY, M_PI_4, &helpQ);

	quat_frameRot(&versX, &versY, &versZ, &versY, &q, &helpQ);

	TEST_ASSERT_EQUAL_QUAT(expected, q);

	/* Rotating frame of reference 90 degrees along z-axis */
	quat_rotQuat(&versZ, M_PI_2, &expected);
	quat_rotQuat(&versZ, M_PI_4, &helpQ);

	quat_frameRot(&versX, &versY, &negVersY, &versX, &q, &helpQ);

	TEST_ASSERT_EQUAL_QUAT(expected, q);
}


TEST(group_quat_frameRot, quat_frameRot_std)
{
	quat_t q;
	frameRotData_t rot = RotExample1;

	quat_frameRot(&rot.v1, &rot.v2, &rot.w1, &rot.w2, &q, NULL);

	/* There are two possible quaternions in this test we don't know which we get, so we cannot compare with expected answer */
	quat_vecRot(&rot.v1, &q);
	quat_vecRot(&rot.v2, &q);

	TEST_ASSERT_EQUAL_VEC(rot.w1, rot.v1);
	TEST_ASSERT_EQUAL_VEC(rot.w2, rot.v2);
}


/*
 * If you combine two rotations - one which rotates `v1` to `w1` and second which rotates `v2` to `w2`,
 * the second rotation have to be along axis parallel to `w1`.
 * In other case the second rotation can break the first one.
 * It can be tricky to calculate this axis when in the second rotation vectors are parallel, but points in opposite directions,
 * because you cannot use cross product to calculate it.
 */
TEST(group_quat_frameRot, quat_frameRot_oneVecAntiparallel)
{
	quat_t q;
	frameRotData_t rot = RotExample2;

	quat_frameRot(&rot.v1, &rot.v2, &rot.w1, &rot.w2, &q, NULL);

	quat_vecRot(&rot.v1, &q);
	quat_vecRot(&rot.v2, &q);

	TEST_ASSERT_EQUAL_VEC(rot.w1, rot.v1);
	TEST_ASSERT_EQUAL_VEC(rot.w2, rot.v2);

	rot.v1 = RotExample2.v1;
	rot.v2 = RotExample2.v2;

	quat_frameRot(&rot.v2, &rot.v1, &rot.w2, &rot.w1, &q, NULL);

	/* There are two possible quaternions in this test we don't know which we get, so we cannot compare with expected answer */
	quat_vecRot(&rot.v1, &q);
	quat_vecRot(&rot.v2, &q);

	TEST_ASSERT_EQUAL_VEC(rot.w1, rot.v1);
	TEST_ASSERT_EQUAL_VEC(rot.w2, rot.v2);
}


/* There are always two quaternions which rotates one frame of reference to another. */
/* We can choose which quaternion we want by passing with quaternion as `help_q` */
TEST(group_quat_frameRot, quat_frameRot_choosingWantedQuat)
{
	quat_t q;
	frameRotData_t rot = RotExample2;

	/* We want first quaternion */
	quat_frameRot(&rot.v1, &rot.v2, &rot.w1, &rot.w2, &q, &rot.q1Closer);
	TEST_ASSERT_QUAT_WITHIN(DELTA, rot.q1, q);

	/* We want second quaternion */
	quat_frameRot(&rot.v1, &rot.v2, &rot.w1, &rot.w2, &q, &rot.q2Closer);
	TEST_ASSERT_QUAT_WITHIN(DELTA, rot.q2, q);
}


TEST_GROUP_RUNNER(group_quat_frameRot)
{
	RUN_TEST_CASE(group_quat_frameRot, quat_frameRot_baseRot);
	RUN_TEST_CASE(group_quat_frameRot, quat_frameRot_std);
	RUN_TEST_CASE(group_quat_frameRot, quat_frameRot_oneVecAntiparallel);
	RUN_TEST_CASE(group_quat_frameRot, quat_frameRot_choosingWantedQuat);
}
