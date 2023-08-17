/*
 * Phoenix-Pilot
 *
 * Unit tests of ekf logging module
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <unity_fixture.h>


void runner(void)
{
	RUN_TEST_GROUP(group_ekf_logs);
}


int main(int argc, char **argv)
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}
