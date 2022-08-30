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


/* Quaternions used for tests */

/* Matrix without zero and one */
static const quat_t Q1 = { .a = 2.0f, .i = 2.0f, .j = 2.0f, .k = 2.0f };


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

	TEST_ASSERT_EQUAL_FLOAT(1.0f, A.a);
	TEST_ASSERT_EQUAL_FLOAT(0.0f, A.i);
	TEST_ASSERT_EQUAL_FLOAT(0.0f, A.j);
	TEST_ASSERT_EQUAL_FLOAT(0.0f, A.k);
}


TEST_GROUP_RUNNER(group_quat_idenWrite)
{
	RUN_TEST_CASE(group_quat_idenWrite, quat_idenWrite_std);
}
