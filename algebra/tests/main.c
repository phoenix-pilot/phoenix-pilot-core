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
	/* Matrix library tests */
	RUN_TEST_GROUP(group_matrix_trp);
	RUN_TEST_GROUP(group_matrix_rowsGet);
	RUN_TEST_GROUP(group_matrix_colsGet);
	RUN_TEST_GROUP(group_matrix_at);
	RUN_TEST_GROUP(group_matrix_bufAlloc);
	RUN_TEST_GROUP(group_matrix_bufFree);
	RUN_TEST_GROUP(group_algebraTests_realTrp);     /* These tests checks tools used in other matrix tests */
	RUN_TEST_GROUP(group_algebraTests_submatCheck); /* These tests checks tools used in other matrix tests */
	RUN_TEST_GROUP(group_matrix_zeroes);
	RUN_TEST_GROUP(group_matrix_diag);
	RUN_TEST_GROUP(group_matrix_times);
	RUN_TEST_GROUP(group_matrix_prod);
	RUN_TEST_GROUP(group_matrix_writeSubmatrix);

	/* Vectors library tests */
	RUN_TEST_GROUP(group_vec_cmp);
	RUN_TEST_GROUP(group_vec_sum);
	RUN_TEST_GROUP(group_vec_add);
	RUN_TEST_GROUP(group_vec_dif);
	RUN_TEST_GROUP(group_vec_sub);

	/* Quaternions library tests */
	RUN_TEST_GROUP(group_quat_cmp);
	RUN_TEST_GROUP(group_quat_idenWrite);
	RUN_TEST_GROUP(group_quat_piWrite);
	RUN_TEST_GROUP(group_quat_add);
}


int main(int argc, char **argv)
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}
