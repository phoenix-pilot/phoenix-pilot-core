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

#include "buffs.h"
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
	quat_t a, i, j, k, q;

	a = i = j = k = q = Q2;

	a.a++;
	i.i++;
	j.j++;
	k.k++;

	q.a++;
	q.i++;
	q.j++;
	q.k++;

	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &a));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &i));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &j));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &k));
	TEST_ASSERT_NOT_EQUAL_INT(QUAT_CMP_OK, quat_cmp(&Q2, &q));
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
	quat_t a = Q1;

	quat_idenWrite(&a);

	TEST_ASSERT_EQUAL_QUAT(QA, a);
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
	quat_t a = Q1;

	quat_piWrite(&a);

	TEST_ASSERT_EQUAL_QUAT(QI, a);
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
	quat_t a = Q2;
	quat_t b = Q3;
	quat_t expected = { .a = a.a + b.a, .i = a.i + b.i, .j = a.j + b.j, .k = a.k + b.k };

	quat_add(&a, &b);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_add, quat_add_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	quat_t expected = { .a = a.a + b.a, .i = a.i + b.i, .j = a.j + b.j, .k = a.k + b.k };

	quat_add(&a, &b);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
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
	quat_t a = Q2;
	quat_t b = Q3;
	quat_t c;
	quat_t expected = { .a = a.a + b.a, .i = a.i + b.i, .j = a.j + b.j, .k = a.k + b.k };

	quat_sum(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(expected, c);
}


TEST(group_quat_sum, quat_sum_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	quat_t c;
	quat_t expected = { .a = a.a + b.a, .i = a.i + b.i, .j = a.j + b.j, .k = a.k + b.k };

	quat_sum(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(expected, c);
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
	quat_t a = Q2;
	quat_t b = Q3;
	quat_t expected = { .a = a.a - b.a, .i = a.i - b.i, .j = a.j - b.j, .k = a.k - b.k };

	quat_sub(&a, &b);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_sub, quat_sub_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	quat_t expected = { .a = a.a - b.a, .i = a.i - b.i, .j = a.j - b.j, .k = a.k - b.k };

	quat_sub(&a, &b);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
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
	quat_t a = Q2;
	quat_t b = Q3;
	quat_t c;
	quat_t expected = { .a = a.a - b.a, .i = a.i - b.i, .j = a.j - b.j, .k = a.k - b.k };

	quat_dif(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(expected, c);
}


TEST(group_quat_dif, quat_dif_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	quat_t c;
	quat_t expected = { .a = a.a - b.a, .i = a.i - b.i, .j = a.j - b.j, .k = a.k - b.k };

	quat_dif(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(expected, c);
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
	quat_t res;

	nqA = nqI = nqJ = nqK = Q0;            /* zero all quaternions */
	nqA.a = nqI.i = nqJ.j = nqK.k = -1.0f; /* set all negative unitary quaternions */

	/* Checking if 1 * 1 is equal to 1 */
	quat_mlt(&QA, &QA, &res);
	TEST_ASSERT_EQUAL_QUAT(QA, res);

	/* Checking if 1 * i is equal to i */
	quat_mlt(&QA, &QI, &res);
	TEST_ASSERT_EQUAL_QUAT(QI, res);

	/* Checking if 1 * j is equal to j */
	quat_mlt(&QA, &QJ, &res);
	TEST_ASSERT_EQUAL_QUAT(QJ, res);

	/* Checking if 1 * k is equal to k */
	quat_mlt(&QA, &QK, &res);
	TEST_ASSERT_EQUAL_QUAT(QK, res);

	/* Checking if i * 1 is equal to i */
	quat_mlt(&QI, &QA, &res);
	TEST_ASSERT_EQUAL_QUAT(QI, res);

	/* Checking if i * i is equal to -1 */
	quat_mlt(&QI, &QI, &res);
	TEST_ASSERT_EQUAL_QUAT(nqA, res);

	/* Checking if i * j is equal to k */
	quat_mlt(&QI, &QJ, &res);
	TEST_ASSERT_EQUAL_QUAT(QK, res);

	/* Checking if i * k is equal to -j */
	quat_mlt(&QI, &QK, &res);
	TEST_ASSERT_EQUAL_QUAT(nqJ, res);

	/* Checking if j * 1 is equal to j */
	quat_mlt(&QJ, &QA, &res);
	TEST_ASSERT_EQUAL_QUAT(QJ, res);

	/* Checking if j * i is equal to -k */
	quat_mlt(&QJ, &QI, &res);
	TEST_ASSERT_EQUAL_QUAT(nqK, res);

	/* Checking if j * j is equal to -1 */
	quat_mlt(&QJ, &QJ, &res);
	TEST_ASSERT_EQUAL_QUAT(nqA, res);

	/* Checking if j * k is equal to i */
	quat_mlt(&QJ, &QK, &res);
	TEST_ASSERT_EQUAL_QUAT(QI, res);

	/* Checking if k * 1 is equal to k */
	quat_mlt(&QK, &QA, &res);
	TEST_ASSERT_EQUAL_QUAT(QK, res);

	/* Checking if k * i is equal to j */
	quat_mlt(&QK, &QI, &res);
	TEST_ASSERT_EQUAL_QUAT(QJ, res);

	/* Checking if k * j is equal to -i */
	quat_mlt(&QK, &QJ, &res);
	TEST_ASSERT_EQUAL_QUAT(nqI, res);

	/* Checking if k * k is equal to -1 */
	quat_mlt(&QK, &QK, &res);
	TEST_ASSERT_EQUAL_QUAT(nqA, res);
}


TEST(group_quat_mlt, quat_mlt_std)
{
	quat_t a = Q2;
	quat_t b = Q3;
	quat_t c;

	quat_mlt(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(Q2timesQ3, c);
}


TEST(group_quat_mlt, quat_mlt_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	quat_t c;

	quat_mlt(&a, &b, &c);

	TEST_ASSERT_EQUAL_QUAT(Q4timesQ5, c);
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
	quat_t a = Q5;
	quat_t expected = {
		.a = a.a * POS_SCALAR,
		.i = a.i * POS_SCALAR,
		.j = a.j * POS_SCALAR,
		.k = a.k * POS_SCALAR
	};

	quat_times(&a, POS_SCALAR);

	TEST_ASSERT_EQUAL_QUAT(expected, a);

	a = Q5;
	expected.a = a.a * NEG_SCALAR;
	expected.i = a.i * NEG_SCALAR;
	expected.j = a.j * NEG_SCALAR;
	expected.k = a.k * NEG_SCALAR;

	quat_times(&a, NEG_SCALAR);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_times, quat_times_infs)
{
	quat_t a = Q5;
	quat_t expected = {
		.a = a.a * INFINITY,
		.i = a.i * INFINITY,
		.j = a.j * INFINITY,
		.k = a.k * INFINITY
	};

	quat_times(&a, INFINITY);

	TEST_ASSERT_EQUAL_QUAT(expected, a);

	a = Q5;
	expected.a = a.a * -INFINITY;
	expected.i = a.i * -INFINITY;
	expected.j = a.j * -INFINITY;
	expected.k = a.k * -INFINITY;

	quat_times(&a, -INFINITY);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_times, quat_times_nan)
{
	quat_t a = Q5;
	quat_t expected = { .a = a.a * NAN, .i = a.i * NAN, .j = a.j * NAN, .k = a.k * NAN };

	quat_times(&a, NAN);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
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
	quat_t a = Q2;
	quat_t expected = { .a = a.a, .i = -a.i, .j = -a.j, .k = -a.k };

	quat_cjg(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
}


TEST(group_quat_cjg, quat_cjg_biggerValues)
{
	quat_t a = Q4;
	quat_t expected = { .a = a.a, .i = -a.i, .j = -a.j, .k = -a.k };

	quat_cjg(&a);

	TEST_ASSERT_EQUAL_QUAT(expected, a);
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
	quat_t a = Q2;
	quat_t b = Q3;
	float expected = a.a * b.a + a.i * b.i + a.j * b.j + a.k * b.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&a, &b));
}


TEST(group_quat_dot, quat_dot_biggerValues)
{
	quat_t a = Q4;
	quat_t b = Q5;
	float expected = a.a * b.a + a.i * b.i + a.j * b.j + a.k * b.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&a, &b));
}


TEST(group_quat_dot, quat_dot_perpendicular)
{
	quat_t a = Q6;
	quat_t b = Q7;
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&a, &b));
	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&b, &a));
}


TEST(group_quat_dot, quat_dot_parallel)
{
	quat_t a = Q2;
	quat_t b = a;
	float expected;

	/* Parallel with common direction */
	quat_times(&b, POS_SCALAR);
	expected = a.a * b.a + a.i * b.i + a.j * b.j + a.k * b.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&a, &b));

	/* Parallel with opposite directions */
	b = a;
	quat_times(&b, NEG_SCALAR);
	expected = a.a * b.a + a.i * b.i + a.j * b.j + a.k * b.k;

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_dot(&a, &b));
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
 * ------------------------        quat_len tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_quat_len);


TEST_SETUP(group_quat_len)
{
}


TEST_TEAR_DOWN(group_quat_len)
{
}


TEST(group_quat_len, quat_len_zeroQuat)
{
	TEST_ASSERT_EQUAL_FLOAT(0, quat_len(&Q0));
}


TEST(group_quat_len, quat_len_baseQuat)
{
	TEST_ASSERT_EQUAL_FLOAT(1, quat_len(&QA));
	TEST_ASSERT_EQUAL_FLOAT(1, quat_len(&QI));
	TEST_ASSERT_EQUAL_FLOAT(1, quat_len(&QJ));
	TEST_ASSERT_EQUAL_FLOAT(1, quat_len(&QK));
}


TEST(group_quat_len, quat_len_std)
{
	quat_t a = Q2;
	float expected = sqrt(a.a * a.a + a.i * a.i + a.j * a.j + a.k * a.k);

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_len(&a));

	/* Checking with negative values */
	quat_times(&a, -1);
	TEST_ASSERT_EQUAL_FLOAT(expected, quat_len(&a));
}


TEST(group_quat_len, quat_len_biggerValues)
{
	quat_t a = Q4;
	float expected = sqrt(a.a * a.a + a.i * a.i + a.j * a.j + a.k * a.k);

	TEST_ASSERT_EQUAL_FLOAT(expected, quat_len(&a));

	/* Checking with negative values */
	quat_times(&a, -1);
	TEST_ASSERT_EQUAL_FLOAT(expected, quat_len(&a));
}


TEST_GROUP_RUNNER(group_quat_len)
{
	RUN_TEST_CASE(group_quat_len, quat_len_zeroQuat);
	RUN_TEST_CASE(group_quat_len, quat_len_baseQuat);
	RUN_TEST_CASE(group_quat_len, quat_len_zeroQuat);
	RUN_TEST_CASE(group_quat_len, quat_len_biggerValues);
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
