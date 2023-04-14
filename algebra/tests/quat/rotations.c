/*
 * Phoenix-Pilot
 *
 * Unit tests for quaternions library
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

#include "buffs.h"
#include "../tools.h"


#define DELTA 1e-7


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
	float roll, pitch, yaw;

	/* quat(1,0,0,0) should give (yaw=0, roll=0, pitch=0) */
	quat_quat2euler(&QA, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, yaw);

	/* quat(0,1,0,0) should give (yaw=0, roll=PI, pitch=0) */
	quat_quat2euler(&QI, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, yaw);

	/* quat(0,0,1,0) should give (yaw=PI, roll=0, pitch=PI) */
	quat_quat2euler(&QJ, &roll, &pitch, &yaw);

	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, roll);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0, pitch);
	TEST_ASSERT_FLOAT_WITHIN(DELTA, M_PI, yaw);

	/* quat(0,0,0,1) should give (yaw=PI, roll=0, pitch=0) */
	quat_quat2euler(&QK, &roll, &pitch, &yaw);

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
	vec_t v1 = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
	vec_t v2 = { .x = -1.0f, .y = 0.0f, .z = 0.0f };
	vec_t v, expected;

	/* Rotation using qA quaternion. Nothing should change */
	v = v1;
	expected = v1;

	quat_vecRot(&v, &QA);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qI quaternion should be equal to rotation about 180 degrees along x-axis. Nothing should change */
	v = v1;
	expected = v1;

	quat_vecRot(&v, &QI);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qJ quaternion should be equal to rotation about 180 degrees along y-axis. */
	v = v1;
	expected = v2;

	quat_vecRot(&v, &QJ);

	TEST_ASSERT_EQUAL_VEC(expected, v);

	/* Rotation using qK quaternion should be equal to rotation about 180 degrees along z-axis. */
	v = v1;
	expected = v2;

	quat_vecRot(&v, &QK);

	TEST_ASSERT_EQUAL_VEC(expected, v);
}


TEST(group_quat_vecRot, quat_vecRot_std)
{
	vec_t v = V1;

	quat_vecRot(&v, &Q8);

	TEST_ASSERT_EQUAL_VEC(V1rotQ8, v);
}


TEST(group_quat_vecRot, quat_vecRot_biggerValues)
{
	vec_t v = V2;

	quat_vecRot(&v, &Q8);

	TEST_ASSERT_EQUAL_VEC(V2rotQ8, v);
}


TEST(group_quat_vecRot, quat_vecRot_zeroVector)
{
	vec_t v1 = V0;

	quat_vecRot(&v1, &Q8);

	TEST_ASSERT_EQUAL_VEC(V0, v1);
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
	quat_t q;

	/* Rotation about 0 angle - quaternion QA */
	quat_rotQuat(&VX, 0.0, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, QA, q);

	/* Rotation about 180 degrees along x-axis - quaternion QI */
	quat_rotQuat(&VX, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, QI, q);

	/* Rotation about 180 degrees along y-axis - quaternion QJ */
	quat_rotQuat(&VY, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, QJ, q);

	/* Rotation about 180 degrees along z-axis - quaternion QK */
	quat_rotQuat(&VZ, M_PI, &q);
	TEST_ASSERT_QUAT_WITHIN(DELTA, QK, q);
}


TEST(group_quat_rotQuat, quat_rotQuat_std)
{
	quat_t q, expected = Q9;

	quat_rotQuat(&V3, Angle, &q);

	TEST_ASSERT_EQUAL_QUAT(expected, q);
}


TEST(group_quat_rotQuat, quat_rotQuat_zeroVector)
{
	vec_t zeroVec = V0;
	quat_t q, expected = QA;

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
	vec_t v1 = V1;
	vec_t v2 = V1rotQ8;

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
	vec_t v1 = V2;
	vec_t v2 = V2rotQ8;

	vec_normalize(&v1);
	vec_normalize(&v2);

	quat_uvec2uvec(&v1, &v2, &q);

	quat_vecRot(&v1, &q);
	TEST_ASSERT_EQUAL_VEC(v2, v1);
}


TEST(group_quat_uvec2uvec, quat_uvec2uvec_parallel)
{
	quat_t q, expected;
	vec_t v1 = V2, v2;

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
	vec_t v1 = V2, v2;

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
	vec_t v1 = V3, v2 = V4;

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
	vec_t v1 = V3, v2 = V5;

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
	const vec_t negVY = { .x = 0.0f, .y = -1.0f, .z = 0.0f };
	quat_t q, expected, helpQ;

	/* Rotating frame of reference 90 degrees along x-axis */
	quat_rotQuat(&VX, M_PI_2, &expected);
	quat_rotQuat(&VX, M_PI_4, &helpQ);

	quat_frameRot(&VX, &VY, &VX, &VZ, &q, &helpQ);

	TEST_ASSERT_EQUAL_QUAT(expected, q);

	/* Rotating frame of reference 90 degrees along y-axis */
	quat_rotQuat(&VY, M_PI_2, &expected);
	quat_rotQuat(&VY, M_PI_4, &helpQ);

	quat_frameRot(&VX, &VY, &VZ, &VY, &q, &helpQ);

	TEST_ASSERT_EQUAL_QUAT(expected, q);

	/* Rotating frame of reference 90 degrees along z-axis */
	quat_rotQuat(&VZ, M_PI_2, &expected);
	quat_rotQuat(&VZ, M_PI_4, &helpQ);

	quat_frameRot(&VX, &VY, &negVY, &VX, &q, &helpQ);

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
