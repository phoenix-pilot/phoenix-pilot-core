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


#define QUAT_CMP_OK 0


/* Quaternions used for tests */

/* Matrix without zero and one */
static const quat_t Q1 = { .a = 2.0f, .i = 2.0f, .j = 2.0f, .k = 2.0f };

static const quat_t Q2 = { .a = 3.0f, .i = 0.0f, .j = 6.0f, .k = 1.0f };
static const quat_t Q3 = { .a = 1.0f, .i = 2.0f, .j = 1.0f, .k = 5.0f };

static const quat_t Q2timesQ3 = { .a = -8.0f, .i = 35.0f, .j = 11.0f, .k = 4.0f };

static const quat_t Q4 = { .a = 815.23f, .i = -818.07f, .j = -451.47f, .k = -546.79f };
static const quat_t Q5 = { .a = 334.23f, .i = -822.81f, .j = 349.42f, .k = 548.18f };

static const quat_t Q4timesQ5 = { .a = 56850.1358f, .i = -1000630.3952f, .j = 1032316.741f, .k = -393184.8904f };


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

	TEST_ASSERT_EQUAL_FLOAT(1.0, A.a);
	TEST_ASSERT_EQUAL_FLOAT(0.0, A.i);
	TEST_ASSERT_EQUAL_FLOAT(0.0, A.j);
	TEST_ASSERT_EQUAL_FLOAT(0.0, A.k);
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

	TEST_ASSERT_EQUAL_FLOAT(0.0, A.a);
	TEST_ASSERT_EQUAL_FLOAT(1.0, A.i);
	TEST_ASSERT_EQUAL_FLOAT(0.0, A.j);
	TEST_ASSERT_EQUAL_FLOAT(0.0, A.k);
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
	quat_t C = A;

	quat_add(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(C.a + B.a, A.a);
	TEST_ASSERT_EQUAL_FLOAT(C.i + B.i, A.i);
	TEST_ASSERT_EQUAL_FLOAT(C.j + B.j, A.j);
	TEST_ASSERT_EQUAL_FLOAT(C.k + B.k, A.k);
}


TEST(group_quat_add, quat_add_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C = A;

	quat_add(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(C.a + B.a, A.a);
	TEST_ASSERT_EQUAL_FLOAT(C.i + B.i, A.i);
	TEST_ASSERT_EQUAL_FLOAT(C.j + B.j, A.j);
	TEST_ASSERT_EQUAL_FLOAT(C.k + B.k, A.k);
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

	quat_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.a + B.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(A.i + B.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(A.j + B.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(A.k + B.k, C.k);
}


TEST(group_quat_sum, quat_sum_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;

	quat_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.a + B.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(A.i + B.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(A.j + B.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(A.k + B.k, C.k);
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
	quat_t C = A;

	quat_sub(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(C.a - B.a, A.a);
	TEST_ASSERT_EQUAL_FLOAT(C.i - B.i, A.i);
	TEST_ASSERT_EQUAL_FLOAT(C.j - B.j, A.j);
	TEST_ASSERT_EQUAL_FLOAT(C.k - B.k, A.k);
}


TEST(group_quat_sub, quat_sub_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C = A;

	quat_sub(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(C.a - B.a, A.a);
	TEST_ASSERT_EQUAL_FLOAT(C.i - B.i, A.i);
	TEST_ASSERT_EQUAL_FLOAT(C.j - B.j, A.j);
	TEST_ASSERT_EQUAL_FLOAT(C.k - B.k, A.k);
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

	quat_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.a - B.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(A.i - B.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(A.j - B.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(A.k - B.k, C.k);
}


TEST(group_quat_dif, quat_dif_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;

	quat_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.a - B.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(A.i - B.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(A.j - B.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(A.k - B.k, C.k);
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
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qA));

	/* Checking if 1 * i is equal to i */
	quat_mlt(&qA, &qI, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qI));

	/* Checking if 1 * j is equal to j */
	quat_mlt(&qA, &qJ, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qJ));

	/* Checking if 1 * k is equal to k */
	quat_mlt(&qA, &qK, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qK));

	/* Checking if i * 1 is equal to i */
	quat_mlt(&qI, &qA, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qI));

	/* Checking if i * i is equal to -1 */
	quat_mlt(&qI, &qI, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqA));

	/* Checking if i * j is equal to k */
	quat_mlt(&qI, &qJ, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qK));

	/* Checking if i * k is equal to -j */
	quat_mlt(&qI, &qK, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqJ));

	/* Checking if j * 1 is equal to j */
	quat_mlt(&qJ, &qA, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qJ));

	/* Checking if j * i is equal to -k */
	quat_mlt(&qJ, &qI, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqK));

	/* Checking if j * j is equal to -1 */
	quat_mlt(&qJ, &qJ, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqA));

	/* Checking if j * k is equal to i */
	quat_mlt(&qJ, &qK, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qI));

	/* Checking if k * 1 is equal to k */
	quat_mlt(&qK, &qA, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qK));

	/* Checking if k * i is equal to j */
	quat_mlt(&qK, &qI, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &qJ));

	/* Checking if k * j is equal to -i */
	quat_mlt(&qK, &qJ, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqI));

	/* Checking if k * k is equal to -1 */
	quat_mlt(&qK, &qK, &Res);
	TEST_ASSERT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Res, &nqA));
}


TEST(group_quat_mlt, quat_mlt_std)
{
	quat_t A = Q2;
	quat_t B = Q3;
	quat_t C;
	quat_t Exp = Q2timesQ3;

	quat_mlt(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(Exp.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(Exp.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(Exp.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(Exp.k, C.k);
}


TEST(group_quat_mlt, quat_mlt_biggerValues)
{
	quat_t A = Q4;
	quat_t B = Q5;
	quat_t C;
	quat_t Exp = Q4timesQ5;

	quat_mlt(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(Exp.a, C.a);
	TEST_ASSERT_EQUAL_FLOAT(Exp.i, C.i);
	TEST_ASSERT_EQUAL_FLOAT(Exp.j, C.j);
	TEST_ASSERT_EQUAL_FLOAT(Exp.k, C.k);
}


TEST_GROUP_RUNNER(group_quat_mlt)
{
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_quatMltTable);
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_std);
	RUN_TEST_CASE(group_quat_mlt, quat_mlt_biggerValues);
}
