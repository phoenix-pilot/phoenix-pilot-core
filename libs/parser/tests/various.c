/*
 * Phoenix-Pilot
 *
 * Unit tests for various functions from parser library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <unity_fixture.h>

#include <parser.h>
#include <hmap.h>

#include <stdlib.h>


#define BUF_LEN 15

#define EXAMPLE_HEADER_NAME1      "struct1"
#define EXAMPLE_HEADER_NAME2      "struct2"
#define TEST_TOO_LONG_HEADER_NAME "too_long_header_to_define"

/* Must be at least 2 */
#define HEADER_NB 5
#define FIELDS_NB 4


static struct {
	parser_t *p;
} various_common;


static int example_converter(const hmap_t *h)
{
	return 0;
}


/* ##############################################################################
 * ----------------------        parser_alloc tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_alloc);

TEST_SETUP(group_parser_alloc)
{
	various_common.p = NULL;
}


TEST_TEAR_DOWN(group_parser_alloc)
{
	parser_free(various_common.p);
}


TEST(group_parser_alloc, parser_alloc_std)
{
	various_common.p = parser_alloc(HEADER_NB, FIELDS_NB);
	TEST_ASSERT_NOT_NULL(various_common.p);
}


TEST(group_parser_alloc, parser_alloc_invalidHeaderNb)
{
	various_common.p = parser_alloc(-HEADER_NB, FIELDS_NB);
	TEST_ASSERT_NULL(various_common.p);

	various_common.p = parser_alloc(0, FIELDS_NB);
	TEST_ASSERT_NULL(various_common.p);
}


TEST(group_parser_alloc, parser_alloc_invalidFieldsNb)
{
	various_common.p = parser_alloc(HEADER_NB, -FIELDS_NB);
	TEST_ASSERT_NULL(various_common.p);

	various_common.p = parser_alloc(HEADER_NB, 0);
	TEST_ASSERT_NULL(various_common.p);
}


TEST_GROUP_RUNNER(group_parser_alloc)
{
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_std);
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_invalidHeaderNb);
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_invalidFieldsNb);
}


/* ##############################################################################
 * -----------------------        parser_free tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_free);

TEST_SETUP(group_parser_free)
{
}


TEST_TEAR_DOWN(group_parser_free)
{
}


TEST(group_parser_free, parser_free_null)
{
	various_common.p = NULL;
	parser_free(various_common.p);

	TEST_ASSERT_NULL(various_common.p);
}


TEST_GROUP_RUNNER(group_parser_free)
{
	RUN_TEST_CASE(group_parser_free, parser_free_null);
}


/* ##############################################################################
 * --------------------        parser_headerAdd tests       ---------------------
 * ############################################################################## */


TEST_GROUP(group_parser_headerAdd);

TEST_SETUP(group_parser_headerAdd)
{
	various_common.p = parser_alloc(HEADER_NB, FIELDS_NB);
	TEST_ASSERT_NOT_NULL(various_common.p);
}


TEST_TEAR_DOWN(group_parser_headerAdd)
{
	parser_free(various_common.p);
}


TEST(group_parser_headerAdd, parser_headerAdd_oneHeader)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, example_converter));
}


TEST(group_parser_headerAdd, parser_headerAdd_multipleHeaders)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, example_converter));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME2, example_converter));
}


TEST(group_parser_headerAdd, parser_headerAdd_passingNull)
{
	/* Parser as NULL */
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(NULL, EXAMPLE_HEADER_NAME1, example_converter));

	/* Header name as NULL */
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(various_common.p, NULL, example_converter));

	/*  Converter as NULL*/
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, NULL));
}


TEST(group_parser_headerAdd, parser_headerAdd_tooManyHeader)
{
	int i;
	char buf[BUF_LEN];

	for (i = 0; i < HEADER_NB; i++) {
		snprintf(buf, BUF_LEN, "%d", i);

		TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(various_common.p, buf, example_converter));
	}

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, example_converter));
}


TEST(group_parser_headerAdd, parser_headerAdd_addingHeaderTwoTimes)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, example_converter));
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(various_common.p, EXAMPLE_HEADER_NAME1, example_converter));
}


TEST(group_parser_headerAdd, parser_headerAdd_tooLongHeader)
{
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_headerAdd(various_common.p, TEST_TOO_LONG_HEADER_NAME, example_converter));
}


TEST_GROUP_RUNNER(group_parser_headerAdd)
{
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_oneHeader);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_multipleHeaders);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_passingNull);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_tooManyHeader);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_addingHeaderTwoTimes);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_tooLongHeader);
}
