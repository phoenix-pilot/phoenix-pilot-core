/*
 * Phoenix-Pilot
 *
 * Unit tests for vectors library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <unity_fixture.h>

#include <vec.h>

#include "buffs.h"
#include "../tools.h"


/* Allowed range around expected value used is some tests */
#define DELTA 5e-6


/* ##############################################################################
 * -------------------------        vec_cmp tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_cmp);


TEST_SETUP(group_vec_cmp)
{
}


TEST_TEAR_DOWN(group_vec_cmp)
{
}


TEST(group_vec_cmp, vec_cmp_stdPass)
{
	TEST_ASSERT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&V3, &V3));
}


TEST(group_vec_cmp, vec_cmp_different)
{
	/* Different x */
	vec_t a = V3;
	a.x++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&a, &V3));

	/* Different y */
	a = V3;
	a.y++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&a, &V3));

	/* Different z */
	a = V3;
	a.z++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&a, &V3));
}


TEST(group_vec_cmp, vec_cmp_diffLNotFails)
{
	vec_t a = V3;
	a.l++;

	TEST_ASSERT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&a, &V3));
}


TEST_GROUP_RUNNER(group_vec_cmp)
{
	RUN_TEST_CASE(group_vec_cmp, vec_cmp_stdPass);
	RUN_TEST_CASE(group_vec_cmp, vec_cmp_different);
	RUN_TEST_CASE(group_vec_cmp, vec_cmp_diffLNotFails);
}


/* ##############################################################################
 * -------------------------        vec_sum tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_sum);


TEST_SETUP(group_vec_sum)
{
}


TEST_TEAR_DOWN(group_vec_sum)
{
}


TEST(group_vec_sum, vec_sum_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t c;
	vec_t expected = { .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };

	vec_sum(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST(group_vec_sum, vec_sum_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t c;
	vec_t expected = { .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };

	vec_sum(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST_GROUP_RUNNER(group_vec_sum)
{
	RUN_TEST_CASE(group_vec_sum, vec_sum_std);
	RUN_TEST_CASE(group_vec_sum, vec_sum_biggerValues);
}


/* ##############################################################################
 * -------------------------        vec_add tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_add);


TEST_SETUP(group_vec_add)
{
}


TEST_TEAR_DOWN(group_vec_add)
{
}


TEST(group_vec_add, vec_add_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t expected = { .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };

	vec_add(&a, &b);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_add, vec_add_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t expected = { .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };

	vec_add(&a, &b);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST_GROUP_RUNNER(group_vec_add)
{
	RUN_TEST_CASE(group_vec_add, vec_add_std);
	RUN_TEST_CASE(group_vec_add, vec_add_biggerValues);
}


/* ##############################################################################
 * -------------------------        vec_dif tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_dif);


TEST_SETUP(group_vec_dif)
{
}


TEST_TEAR_DOWN(group_vec_dif)
{
}


TEST(group_vec_dif, vec_dif_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t c;
	vec_t expected = { .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };

	vec_dif(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST(group_vec_dif, vec_dif_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t c;
	vec_t expected = { .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };

	vec_dif(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST_GROUP_RUNNER(group_vec_dif)
{
	RUN_TEST_CASE(group_vec_dif, vec_dif_std);
	RUN_TEST_CASE(group_vec_dif, vec_dif_biggerValues);
}


/* ##############################################################################
 * -------------------------        vec_sub tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_sub);


TEST_SETUP(group_vec_sub)
{
}


TEST_TEAR_DOWN(group_vec_sub)
{
}


TEST(group_vec_sub, vec_sub_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t expected = { .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };

	vec_sub(&a, &b);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_sub, vec_sub_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t expected = { .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };

	vec_sub(&a, &b);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST_GROUP_RUNNER(group_vec_sub)
{
	RUN_TEST_CASE(group_vec_sub, vec_sub_std);
	RUN_TEST_CASE(group_vec_sub, vec_sub_biggerValues);
}


/* ##############################################################################
 * ------------------------        vec_times tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_times);


TEST_SETUP(group_vec_times)
{
}


TEST_TEAR_DOWN(group_vec_times)
{
}


TEST(group_vec_times, vec_times_std)
{
	vec_t a = V3;
	vec_t expected = {
		.x = a.x * POS_SCALAR,
		.y = a.y * POS_SCALAR,
		.z = a.z * POS_SCALAR
	};

	vec_times(&a, POS_SCALAR);

	TEST_ASSERT_EQUAL_VEC(expected, a);

	a = V3;
	expected.x = a.x * NEG_SCALAR;
	expected.y = a.y * NEG_SCALAR;
	expected.z = a.z * NEG_SCALAR;

	vec_times(&a, NEG_SCALAR);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_times, vec_times_infs)
{
	vec_t a = V3;
	vec_t expected = {
		.x = a.x * INFINITY,
		.y = a.y * INFINITY,
		.z = a.z * INFINITY
	};

	vec_times(&a, INFINITY);

	TEST_ASSERT_EQUAL_VEC(expected, a)

	a = V3;
	expected.x = a.x * -INFINITY;
	expected.y = a.y * -INFINITY;
	expected.z = a.z * -INFINITY;

	vec_times(&a, -INFINITY);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_times, vec_times_nan)
{
	vec_t a = V3;
	vec_t expected = {
		.x = a.x * NAN,
		.y = a.y * NAN,
		.z = a.z * NAN
	};

	vec_times(&a, NAN);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST_GROUP_RUNNER(group_vec_times)
{
	RUN_TEST_CASE(group_vec_times, vec_times_std);
	RUN_TEST_CASE(group_vec_times, vec_times_infs);
	RUN_TEST_CASE(group_vec_times, vec_times_nan);
}


/* ##############################################################################
 * ------------------------        vec_cross tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_cross);


TEST_SETUP(group_vec_cross)
{
}


TEST_TEAR_DOWN(group_vec_cross)
{
}


TEST(group_vec_cross, vec_cross_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t c;
	vec_t expected = {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x
	};

	vec_cross(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST(group_vec_cross, vec_cross_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t c;
	vec_t expected = {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x
	};

	vec_cross(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST(group_vec_cross, vec_cross_perpendicular)
{
	vec_t a = V5;
	vec_t b = V6;
	vec_t c;
	vec_t expected = {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x
	};

	vec_cross(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);

	expected.x = b.y * a.z - b.z * a.y;
	expected.y = b.z * a.x - b.x * a.z;
	expected.z = b.x * a.y - b.y * a.x;

	vec_cross(&b, &a, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST(group_vec_cross, vec_cross_parallel)
{
	vec_t a = V2;
	vec_t b = a;
	vec_t c;
	vec_t expected = { .x = 0.0, .y = 0.0, .z = 0.0 };

	/* Parallel with common direction */
	vec_times(&b, POS_SCALAR);

	vec_cross(&a, &b, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);

	/* Parallel with opposite directions */
	b = a;
	vec_times(&b, NEG_SCALAR);

	vec_cross(&b, &a, &c);

	TEST_ASSERT_EQUAL_VEC(expected, c);
}


TEST_GROUP_RUNNER(group_vec_cross)
{
	RUN_TEST_CASE(group_vec_cross, vec_cross_std);
	RUN_TEST_CASE(group_vec_cross, vec_cross_biggerValues);
	RUN_TEST_CASE(group_vec_cross, vec_cross_perpendicular);
	RUN_TEST_CASE(group_vec_cross, vec_cross_parallel);
}


/* ##############################################################################
 * -------------------------        vec_dot tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_dot);


TEST_SETUP(group_vec_dot)
{
}


TEST_TEAR_DOWN(group_vec_dot)
{
}


TEST(group_vec_dot, vec_dot_std)
{
	vec_t a = V1;
	vec_t b = V2;
	float expected = a.x * b.x + a.y * b.y + a.z * b.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&a, &b));
}


TEST(group_vec_dot, vec_dot_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	float expected = a.x * b.x + a.y * b.y + a.z * b.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&a, &b));
}


TEST(group_vec_dot, vec_dot_perpendicular)
{
	vec_t a = V5;
	vec_t b = V6;
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&a, &b));
	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&b, &a));
}


TEST(group_vec_dot, vec_dot_parallel)
{
	vec_t a = V2;
	vec_t b = a;
	float expected;

	/* Parallel with common direction */
	vec_times(&b, POS_SCALAR);
	expected = a.x * b.x + a.y * b.y + a.z * b.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&a, &b));

	/* Parallel with opposite directions */
	b = a;
	vec_times(&b, NEG_SCALAR);
	expected = a.x * b.x + a.y * b.y + a.z * b.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&a, &b));
}


TEST_GROUP_RUNNER(group_vec_dot)
{
	RUN_TEST_CASE(group_vec_dot, vec_dot_std);
	RUN_TEST_CASE(group_vec_dot, vec_dot_biggerValues);
	RUN_TEST_CASE(group_vec_dot, vec_dot_perpendicular);
	RUN_TEST_CASE(group_vec_dot, vec_dot_parallel);
}


/* ##############################################################################
 * -------------------------        vec_len tests       -------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_len);


TEST_SETUP(group_vec_len)
{
}


TEST_TEAR_DOWN(group_vec_len)
{
}


TEST(group_vec_len, vec_len_std)
{
	vec_t a = V1;
	float expected = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&a));
}


TEST(group_vec_len, vec_len_biggerValues)
{
	vec_t a = V3;
	float expected = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&a));
}


TEST(group_vec_len, vec_len_zeroLen)
{
	vec_t a = V0;
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&a));
}


TEST_GROUP_RUNNER(group_vec_len)
{
	RUN_TEST_CASE(group_vec_len, vec_len_std);
	RUN_TEST_CASE(group_vec_len, vec_len_biggerValues);
	RUN_TEST_CASE(group_vec_len, vec_len_zeroLen);
}


/* ##############################################################################
 * ------------------------        vec_normal tests       ------------------------
 * ############################################################################## */


TEST_GROUP(group_vec_normal);


TEST_SETUP(group_vec_normal)
{
}


TEST_TEAR_DOWN(group_vec_normal)
{
}


TEST(group_vec_normal, vec_normal_std)
{
	vec_t a = V1;
	vec_t b = V2;
	vec_t c;

	vec_normal(&a, &b, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);
}


TEST(group_vec_normal, vec_normal_biggerValues)
{
	vec_t a = V3;
	vec_t b = V4;
	vec_t c;

	vec_normal(&a, &b, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);
}


TEST(group_vec_normal, vec_normal_perpendicular)
{
	vec_t a = V5;
	vec_t b = V6;
	vec_t c;

	vec_normal(&a, &b, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);
}


TEST(group_vec_normal, vec_normal_parallel)
{
	vec_t a = V2;
	vec_t b = a;
	vec_t c;

	/* Parallel with common direction */
	vec_times(&b, POS_SCALAR);

	vec_normal(&a, &b, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);

	/* Parallel with opposite directions */
	b = a;
	vec_times(&b, NEG_SCALAR);

	vec_normal(&b, &a, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);
}


/* When one (and only one) argument is zero vector, then function should return vector perpendicular to non zero vector */
TEST(group_vec_normal, vec_normal_singleZeroVec)
{
	vec_t a = V3;
	vec_t b = V0;
	vec_t c;

	vec_normal(&a, &b, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);

	vec_normal(&b, &a, &c);

	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, a, c);
	TEST_ASSERT_PERPENDICULAR_VEC(DELTA, b, c);
	TEST_ASSERT_UNIT_VEC(c);
}


/* When both arguments are zero vectors, then function should return zero vector */
TEST(group_vec_normal, vec_normal_bothZeroVectors)
{
	vec_t c;

	vec_normal(&V0, &V0, &c);

	TEST_ASSERT_EQUAL_VEC(V0, c);
}


TEST_GROUP_RUNNER(group_vec_normal)
{
	RUN_TEST_CASE(group_vec_normal, vec_normal_std);
	RUN_TEST_CASE(group_vec_normal, vec_normal_biggerValues);
	RUN_TEST_CASE(group_vec_normal, vec_normal_perpendicular);
	RUN_TEST_CASE(group_vec_normal, vec_normal_parallel);
	RUN_TEST_CASE(group_vec_normal, vec_normal_singleZeroVec);
	RUN_TEST_CASE(group_vec_normal, vec_normal_bothZeroVectors);
}


/* ##############################################################################
 * ----------------------        vec_normalize tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_vec_normalize);


TEST_SETUP(group_vec_normalize)
{
}


TEST_TEAR_DOWN(group_vec_normalize)
{
}


TEST(group_vec_normalize, vec_normalize_lessThanUnit)
{
	vec_t a = V7;
	float len = vec_len(&a);
	vec_t expected = {
		.x = a.x / len,
		.y = a.y / len,
		.z = a.z / len
	};

	vec_normalize(&a);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_normalize, vec_normalize_moreThanUnit)
{
	vec_t a = V3;
	float len = vec_len(&a);
	vec_t expected = {
		.x = a.x / len,
		.y = a.y / len,
		.z = a.z / len
	};

	vec_normalize(&a);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_normalize, vec_normalize_equalUnit)
{
	vec_t a = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
	vec_t expected = a;

	vec_normalize(&a);

	TEST_ASSERT_EQUAL_VEC(expected, a);

	a.x = -1;
	expected = a;

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST(group_vec_normalize, vec_normalize_zeroVec)
{
	vec_t a = V0;
	vec_t expected = V0;

	vec_normalize(&a);

	TEST_ASSERT_EQUAL_VEC(expected, a);
}


TEST_GROUP_RUNNER(group_vec_normalize)
{
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_lessThanUnit);
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_moreThanUnit);
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_equalUnit);
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_zeroVec);
}
