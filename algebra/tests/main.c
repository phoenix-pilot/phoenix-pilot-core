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
	RUN_TEST_GROUP(group_matrix_trp);
	RUN_TEST_GROUP(group_matrix_rowsGet);
	RUN_TEST_GROUP(group_matrix_colsGet);
	RUN_TEST_GROUP(group_matrix_at);
	RUN_TEST_GROUP(group_matrix_bufAlloc);
	RUN_TEST_GROUP(group_matrix_bufFree);
	RUN_TEST_GROUP(group_matrix_zeroes);
	RUN_TEST_GROUP(group_matrix_diag);
}


int main(int argc, char **argv)
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}
