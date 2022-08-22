/*
 * Phoenix-Pilot
 *
 * Unit tests for algebra operations
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <unity_fixture.h>


void runner(void)
{
	RUN_TEST_GROUP(group_matrix_at);
	RUN_TEST_GROUP(group_matrix_zeroes);
}


int main(int argc, char **argv)
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}