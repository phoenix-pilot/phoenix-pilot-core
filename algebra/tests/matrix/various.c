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

#include <string.h>

#include <unity_fixture.h>

#include <matrix.h>

/* Creating matrix for testing */
#define ROWS 10
#define COLS 5

static float buf[ROWS * COLS];
static matrix_t A = { .data = buf, .cols = COLS, .rows = ROWS };


TEST_GROUP(group_matrix_zeroes);


TEST_SETUP(group_matrix_zeroes)
{
	memset(buf, 1, sizeof(float) * ROWS * COLS);
}


TEST_TEAR_DOWN(group_matrix_zeroes)
{
}


TEST(group_matrix_zeroes, matrix_zeroes_std)
{
	int i;

	matrix_zeroes(&A);

	/* #TODO this should be done using function 'at' later */
	for (i = 0; i < ROWS * COLS; i++) {
		TEST_ASSERT_EQUAL_FLOAT(0.0, A.data[i]);
	}
}


TEST_GROUP_RUNNER(group_matrix_zeroes)
{
	RUN_TEST_CASE(group_matrix_zeroes, matrix_zeroes_std);
}
