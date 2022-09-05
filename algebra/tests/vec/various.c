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


/* Values used for vectors library tests */

/* Small values */
static const vec_t V1 = { .x = 1.0f, .y = 2.0f, .z = 3.0f };
static const vec_t V2 = { .x = 4.0f, .y = 5.0f, .z = 6.0f };

/* More complicated values */
static const vec_t V3 = { .x = -261.48f, .y = 731.11f, .z = -919.51f };
static const vec_t V4 = { .x = 613.36f, .y = -708.58f, .z = -150.27f };


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

	vec_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.x + B.x, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.y + B.y, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.z + B.z, C.z);
}


TEST(group_vec_sum, vec_sum_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;

	vec_sum(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.x + B.x, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.y + B.y, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.z + B.z, C.z);
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
	vec_t cpyA = A;

	vec_add(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(cpyA.x + B.x, A.x);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.y + B.y, A.y);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.z + B.z, A.z);
}


TEST(group_vec_add, vec_add_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t cpyA = A;

	vec_add(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(cpyA.x + B.x, A.x);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.y + B.y, A.y);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.z + B.z, A.z);
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

	vec_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.x - B.x, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.y - B.y, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.z - B.z, C.z);
}


TEST(group_vec_dif, vec_dif_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;

	vec_dif(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.x - B.x, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.y - B.y, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.z - B.z, C.z);
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
	vec_t cpyA = A;

	vec_sub(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(cpyA.x - B.x, A.x);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.y - B.y, A.y);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.z - B.z, A.z);
}


TEST(group_vec_sub, vec_sub_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t cpyA = A;

	vec_sub(&A, &B);

	TEST_ASSERT_EQUAL_FLOAT(cpyA.x - B.x, A.x);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.y - B.y, A.y);
	TEST_ASSERT_EQUAL_FLOAT(cpyA.z - B.z, A.z);
}


TEST_GROUP_RUNNER(group_vec_sub)
{
	RUN_TEST_CASE(group_vec_sub, vec_sub_std);
	RUN_TEST_CASE(group_vec_sub, vec_sub_biggerValues);
}
