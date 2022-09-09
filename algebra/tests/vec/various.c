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

/* More complicated values */
static const vec_t V3 = { .x = -261.48f, .y = 731.11f, .z = -919.51f };
static const vec_t V4 = { .x = 613.36f, .y = -708.58f, .z = -150.27f };

/* This vectors are perpendicular to each other */
static const vec_t V5 = { .x = 0.0f, .y = 1.0f, .z = 4.0f };
static const vec_t V6 = { .x = 5.0f, .y = -8.0f, .z = 2.0f };


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

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.y * B.z - A.z * B.y, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.z * B.x - A.x * B.z, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.x * B.y - A.y * B.x, C.z);
}


TEST(group_vec_cross, vec_cross_biggerValues)
{
	vec_t A = V3;
	vec_t B = V4;
	vec_t C;

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.y * B.z - A.z * B.y, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.z * B.x - A.x * B.z, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.x * B.y - A.y * B.x, C.z);
}


TEST(group_vec_cross, vec_cross_perpendicular)
{
	vec_t A = V5;
	vec_t B = V6;
	vec_t C;

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(A.y * B.z - A.z * B.y, C.x);
	TEST_ASSERT_EQUAL_FLOAT(A.z * B.x - A.x * B.z, C.y);
	TEST_ASSERT_EQUAL_FLOAT(A.x * B.y - A.y * B.x, C.z);

	vec_cross(&B, &A, &C);

	TEST_ASSERT_EQUAL_FLOAT(B.y * A.z - B.z * A.y, C.x);
	TEST_ASSERT_EQUAL_FLOAT(B.z * A.x - B.x * A.z, C.y);
	TEST_ASSERT_EQUAL_FLOAT(B.x * A.y - B.y * A.x, C.z);
}


TEST(group_vec_cross, vec_cross_parallel)
{
	vec_t A = V2;
	vec_t B = A;
	vec_t C;

	/* Parallel with common direction */
	vec_times(&B, POS_SCALAR);

	vec_cross(&A, &B, &C);

	TEST_ASSERT_EQUAL_FLOAT(0.0, C.x);
	TEST_ASSERT_EQUAL_FLOAT(0.0, C.y);
	TEST_ASSERT_EQUAL_FLOAT(0.0, C.z);

	/* Parallel with opposite directions */
	B = A;
	vec_times(&B, NEG_SCALAR);

	vec_cross(&B, &A, &C);

	TEST_ASSERT_EQUAL_FLOAT(0.0, C.x);
	TEST_ASSERT_EQUAL_FLOAT(0.0, C.y);
	TEST_ASSERT_EQUAL_FLOAT(0.0, C.z);
}


TEST_GROUP_RUNNER(group_vec_cross)
{
	RUN_TEST_CASE(group_vec_cross, vec_cross_std);
	RUN_TEST_CASE(group_vec_cross, vec_cross_biggerValues);
	RUN_TEST_CASE(group_vec_cross, vec_cross_perpendicular);
	RUN_TEST_CASE(group_vec_cross, vec_cross_parallel);
}
