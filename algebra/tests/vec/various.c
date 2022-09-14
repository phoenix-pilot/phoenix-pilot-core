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


#define TEST_ASSERT_EQUAL_VEC(expected, actual) \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).x, (actual).x, "Different `x` part of vectors"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).y, (actual).y, "Different `y` part of vectors"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).z, (actual).z, "Different `z` part of vectors");

#define VEC_CMP_OK 0

#define POS_SCALAR 2.5
#define NEG_SCALAR -3.75


/* Values used for vectors library tests */

/* Small values */
static const vec_t V1 = { .x = 1.0f, .y = 2.0f, .z = 3.0f };
static const vec_t V2 = { .x = 4.0f, .y = 5.0f, .z = 6.0f };

/* More complicated values. Length of this vectors must be bigger than 1 */
static const vec_t V3 = { .x = -261.48f, .y = 731.11f, .z = -919.51f };
static const vec_t V4 = { .x = 613.36f, .y = -708.58f, .z = -150.27f };

/* This vectors are perpendicular to each other */
static const vec_t V5 = { .x = 0.0f, .y = 1.0f, .z = 4.0f };
static const vec_t V6 = { .x = 5.0f, .y = -8.0f, .z = 2.0f };

/* Length of this vector must be smaller than 1 */
static const vec_t V7 = { .x = 0.25f, .y = 0.5f, .z = 0.5f };


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
	vec_t A = V3;
	A.x++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&A, &V3));

	/* Different y */
	A = V3;
	A.y++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&A, &V3));

	/* Different z */
	A = V3;
	A.z++;

	TEST_ASSERT_NOT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&A, &V3));
}


TEST(group_vec_cmp, vec_cmp_diffLNotFails)
{
	vec_t A = V3;
	A.l++;

	TEST_ASSERT_EQUAL_INT(VEC_CMP_OK, vec_cmp(&A, &V3));
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
	vec_t A = V1;
	vec_t B = V2;
	vec_t C;
	vec_t Exp = { .x = A.x + B.x, .y = A.y + B.y, .z = A.z + B.z };

	vec_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
}


TEST(group_vec_sum, vec_sum_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;
	vec_t Exp = { .x = A.x + B.x, .y = A.y + B.y, .z = A.z + B.z };

	vec_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
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
	vec_t A = V1;
	vec_t B = V2;
	vec_t Exp = { .x = A.x + B.x, .y = A.y + B.y, .z = A.z + B.z };

	vec_add(&A, &B);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
}


TEST(group_vec_add, vec_add_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t Exp = { .x = A.x + B.x, .y = A.y + B.y, .z = A.z + B.z };

	vec_add(&A, &B);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
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
	vec_t A = V1;
	vec_t B = V2;
	vec_t C;
	vec_t Exp = { .x = A.x - B.x, .y = A.y - B.y, .z = A.z - B.z };

	vec_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
}


TEST(group_vec_dif, vec_dif_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;
	vec_t Exp = { .x = A.x - B.x, .y = A.y - B.y, .z = A.z - B.z };

	vec_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
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
	vec_t A = V1;
	vec_t B = V2;
	vec_t Exp = { .x = A.x - B.x, .y = A.y - B.y, .z = A.z - B.z };

	vec_sub(&A, &B);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
}


TEST(group_vec_sub, vec_sub_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t Exp = { .x = A.x - B.x, .y = A.y - B.y, .z = A.z - B.z };

	vec_sub(&A, &B);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
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
	vec_t A = V3;
	vec_t Exp = {
		.x = A.x * POS_SCALAR,
		.y = A.y * POS_SCALAR,
		.z = A.z * POS_SCALAR
	};

	vec_times(&A, POS_SCALAR);

	TEST_ASSERT_EQUAL_VEC(Exp, A);

	A = V3;
	Exp.x = A.x * NEG_SCALAR;
	Exp.y = A.y * NEG_SCALAR;
	Exp.z = A.z * NEG_SCALAR;

	vec_times(&A, NEG_SCALAR);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
}


TEST(group_vec_times, vec_times_infs)
{
	vec_t A = V3;
	vec_t Exp = {
		.x = A.x * INFINITY,
		.y = A.y * INFINITY,
		.z = A.z * INFINITY
	};

	vec_times(&A, INFINITY);

	TEST_ASSERT_EQUAL_VEC(Exp, A)

	A = V3;
	Exp.x = A.x * -INFINITY;
	Exp.y = A.y * -INFINITY;
	Exp.z = A.z * -INFINITY;

	vec_times(&A, -INFINITY);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
}


TEST(group_vec_times, vec_times_nan)
{
	vec_t A = V3;
	vec_t Exp = {
		.x = A.x * NAN,
		.y = A.y * NAN,
		.z = A.z * NAN
	};

	vec_times(&A, NAN);

	TEST_ASSERT_EQUAL_VEC(Exp, A);
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
	vec_t A = V1;
	vec_t B = V2;
	vec_t C;
	vec_t Exp = {
		.x = A.y * B.z - A.z * B.y,
		.y = A.z * B.x - A.x * B.z,
		.z = A.x * B.y - A.y * B.x
	};

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
}


TEST(group_vec_cross, vec_cross_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;
	vec_t Exp = {
		.x = A.y * B.z - A.z * B.y,
		.y = A.z * B.x - A.x * B.z,
		.z = A.x * B.y - A.y * B.x
	};

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
}


TEST(group_vec_cross, vec_cross_perpendicular)
{
	vec_t A = V5;
	vec_t B = V6;
	vec_t C;
	vec_t Exp = {
		.x = A.y * B.z - A.z * B.y,
		.y = A.z * B.x - A.x * B.z,
		.z = A.x * B.y - A.y * B.x
	};

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);

	Exp.x = B.y * A.z - B.z * A.y;
	Exp.y = B.z * A.x - B.x * A.z;
	Exp.z = B.x * A.y - B.y * A.x;

	vec_cross(&B, &A, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
}


TEST(group_vec_cross, vec_cross_parallel)
{
	vec_t A = V2;
	vec_t B = A;
	vec_t C;
	vec_t Exp = { .x = 0.0, .y = 0.0, .z = 0.0 };

	/* Parallel with common direction */
	vec_times(&B, POS_SCALAR);

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);

	/* Parallel with opposite directions */
	B = A;
	vec_times(&B, NEG_SCALAR);

	vec_cross(&B, &A, &C);

	TEST_ASSERT_EQUAL_VEC(Exp, C);
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
	vec_t A = V1;
	vec_t B = V2;
	float expected = A.x * B.x + A.y * B.y + A.z * B.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&A, &B));
}


TEST(group_vec_dot, vec_dot_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	float expected = A.x * B.x + A.y * B.y + A.z * B.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&A, &B));
}


TEST(group_vec_dot, vec_dot_perpendicular)
{
	vec_t A = V5;
	vec_t B = V6;
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&A, &B));
	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&B, &A));
}


TEST(group_vec_dot, vec_dot_parallel)
{
	vec_t A = V2;
	vec_t B = A;
	float expected;

	/* Parallel with common direction */
	vec_times(&B, POS_SCALAR);
	expected = A.x * B.x + A.y * B.y + A.z * B.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&A, &B));

	/* Parallel with opposite directions */
	B = A;
	vec_times(&B, NEG_SCALAR);
	expected = A.x * B.x + A.y * B.y + A.z * B.z;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_dot(&A, &B));
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
	vec_t A = V1;
	float expected = sqrt(A.x * A.x + A.y * A.y + A.z * A.z);

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&A));
}


TEST(group_vec_len, vec_len_biggerValues)
{
	vec_t A = V3;
	float expected = sqrt(A.x * A.x + A.y * A.y + A.z * A.z);

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&A));
}


TEST(group_vec_len, vec_len_zeroLen)
{
	vec_t A = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
	float expected = 0;

	TEST_ASSERT_EQUAL_FLOAT(expected, vec_len(&A));
}


TEST_GROUP_RUNNER(group_vec_len)
{
	RUN_TEST_CASE(group_vec_len, vec_len_std);
	RUN_TEST_CASE(group_vec_len, vec_len_biggerValues);
	RUN_TEST_CASE(group_vec_len, vec_len_zeroLen);
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


TEST_GROUP_RUNNER(group_vec_normalize)
{
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_lessThanUnit);
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_moreThanUnit);
	RUN_TEST_CASE(group_vec_normalize, vec_normalize_equalUnit);
}
