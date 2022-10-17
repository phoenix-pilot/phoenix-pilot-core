/*
 * Phoenix-Pilot
 *
 * Unit tests for parser library
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
	/* Tests from `various.c` */
	RUN_TEST_GROUP(group_parser_alloc);
	RUN_TEST_GROUP(group_parser_free);
	RUN_TEST_GROUP(group_parser_headerAdd);

	/* Tests from `parsing.c` */
	RUN_TEST_GROUP(group_parser_execute);
	RUN_TEST_GROUP(group_parser_clear);
}


int main(int argc, char **argv)
{
	UnityMain(argc, (const char **)argv, runner);

	return 0;
}
