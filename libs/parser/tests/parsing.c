/*
 * Phoenix-Pilot
 *
 * Unit tests for functions from parser library
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

#include <stdio.h>
#include <string.h>


#define HEADERS_NB 4

/* Have to be smaller than 7. */
/* In other case change `too_many_fields` file */
#define TEST_MAX_FIELDS_NB 3

#define TEST_STRUCT1_HEADER_NAME "STRUCT1"
#define TEST_STRUCT2_HEADER_NAME "STRUCT2"

#define INT1_FIELD_NAME   "INT1"
#define INT2_FIELD_NAME   "INT2"
#define FLOAT_FIELD_NAME  "FLOAT"
#define STRING_FIELD_NAME "STRING"

static struct {
	parser_t *p;

	int invCnt;   /* number of converter invocations */
	int corrData; /* converter changes this field to non zero if parser passed hash map different than expected */
} parsing_common;


/* Checks if hash map have key equal to `fieldName` and it's value is the same as `expValue`. If succeeded returns 0. In other case returns -1. */
static int test_fieldCheck(const hmap_t *h, const char *fieldName, const char *expValue)
{
	char *data = hmap_get(h, fieldName);
	if (data == NULL) {
		fprintf(stderr, "Converter: value of %s field was NULL\n", INT1_FIELD_NAME);

		return -1;
	}
	if (strcmp(expValue, data) != 0) {
		fprintf(stderr, "Converter: incorrect value. Expected %s was %s\n", expValue, data);

		return -1;
	}

	return 0;
}


/* Checks if hash map have size equal to `expSize`. If succeeded returns 0. In other case returns -1. */
static int test_hmapSizeCheck(const hmap_t *h, int expSize)
{
	if (h->size != expSize) {
		fprintf(stderr, "Converter: Incorrect number of fields in hash map. Expected %d was %d.\n", expSize, (unsigned int)h->size);

		return -1;
	}

	return 0;
}


/* ##############################################################################
 * ---------------------        parser_execute tests       ----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_execute);


TEST_SETUP(group_parser_execute)
{
	/* Setting up the parser */
	parsing_common.p = parser_alloc(HEADERS_NB, TEST_MAX_FIELDS_NB);
	TEST_ASSERT_NOT_NULL(parsing_common.p);

	parsing_common.corrData = 0;
	parsing_common.invCnt = 0;
}


TEST_TEAR_DOWN(group_parser_execute)
{
	parser_free(parsing_common.p);
}


static int converter_oneHeader(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_hmapSizeCheck(h, 3);
	res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
	res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
	res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_oneHeader)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_oneHeader));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/one_header", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_multipleHeaders(const hmap_t *h)
{
	int res = 0;

	switch (++parsing_common.invCnt) {
		case 1:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");
			break;
		case 2:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "16");
			break;
		case 3:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "27");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "56.25");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "STR");
			break;
		case 4:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "25");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "123");
			break;
		default:
			fprintf(stderr, "Too many converter invocations");
			res += -1;
			break;
	}

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_multipleHeaders)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_multipleHeaders));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_multipleHeaders));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/multiple_headers", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(4, parsing_common.invCnt);
}


static int converter_comments(const hmap_t *h)
{
	int res = 0;

	switch (++parsing_common.invCnt) {
		case 1:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");
			break;
		case 2:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "16");
			break;
		default:
			fprintf(stderr, "Too many converter invocations");
			res += -1;
			break;
	}

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_comments)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_comments));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_comments));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/comments", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(2, parsing_common.invCnt);
}


static int converter_spaces(const hmap_t *h)
{
	int res = 0;

	switch (++parsing_common.invCnt) {
		case 1:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "123");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "lorem_ipsum");
			break;
		case 2:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "12");
			break;
		default:
			fprintf(stderr, "Too many converter invocations");
			res += -1;
			break;
	}

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_spaces)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_spaces));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_spaces));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/spaces", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(2, parsing_common.invCnt);
}


static int converter_signs(const hmap_t *h)
{
	int res = 0;

	switch (++parsing_common.invCnt) {
		case 1:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "123");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "-25");
			break;
		case 2:
			res += test_hmapSizeCheck(h, 2);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "+123");
			res += test_fieldCheck(h, INT2_FIELD_NAME, "25");
			break;
		default:
			fprintf(stderr, "Too many converter invocations");
			res += -1;
			break;
	}

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_signs)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_signs));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/signs", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(2, parsing_common.invCnt);
}


static int converter_ignoreUnknownHeaders(const hmap_t *h)
{
	int res = 0;

	switch (++parsing_common.invCnt) {
		case 1:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");
			break;
		case 2:
			res += test_hmapSizeCheck(h, 3);
			res += test_fieldCheck(h, INT1_FIELD_NAME, "27");
			res += test_fieldCheck(h, FLOAT_FIELD_NAME, "56.25");
			res += test_fieldCheck(h, STRING_FIELD_NAME, "STR");
			break;
		default:
			fprintf(stderr, "Too many converter invocations");
			res += -1;
			break;
	}

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_ignoreUnknowHeaders)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_ignoreUnknownHeaders));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/multiple_headers", PARSER_IGN_UNKNOWN_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(2, parsing_common.invCnt);
}


TEST(group_parser_execute, parser_execute_emptyFile)
{
	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/empty_file", PARSER_EXEC_ALL_HEADERS));
}


static int converter_unspecifiedHeader(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
	res += test_fieldCheck(h, INT2_FIELD_NAME, "16");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_unspecifiedHeader)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_unspecifiedHeader));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_unspecifiedHeader));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/unspecified_header", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_tooManyFields(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
	res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
	res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_tooManyFields)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_tooManyFields));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_tooManyFields));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/too_many_fields", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_redundantFields(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_fieldCheck(h, INT1_FIELD_NAME, "5");
	res += test_fieldCheck(h, FLOAT_FIELD_NAME, "2.5");
	res += test_fieldCheck(h, STRING_FIELD_NAME, "TEST");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_redundantFields)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_redundantFields));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_redundantFields));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/redundant_fields", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_fieldWithoutHeader(const hmap_t *h)
{
	parsing_common.corrData = -1;
	parsing_common.invCnt++;

	return -1;
}


TEST(group_parser_execute, parser_execute_fieldWithoutHeader1)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_fieldWithoutHeader));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_fieldWithoutHeader));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/field_without_header", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(0, parsing_common.invCnt);
}


TEST(group_parser_execute, parser_execute_fieldWithoutHeader2)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_fieldWithoutHeader));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/field_without_header", PARSER_IGN_UNKNOWN_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(0, parsing_common.invCnt);
}


static int converter_failAtTheBeginning(const hmap_t *h)
{
	parsing_common.invCnt++;

	return -1;
}


/* Converter always fails */
TEST(group_parser_execute, parser_execute_failAtTheBeginning)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_failAtTheBeginning));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_failAtTheBeginning));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/multiple_headers", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_failInTheMiddle(const hmap_t *h)
{
	if (++parsing_common.invCnt > 2) {
		return -1;
	}

	return 0;
}


/* Converter fails During third incocation */
TEST(group_parser_execute, parser_execute_failInTheMiddle)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_failInTheMiddle));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_failInTheMiddle));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/multiple_headers", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(3, parsing_common.invCnt);
}


static int converter_tooLongField(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
	res += test_fieldCheck(h, INT2_FIELD_NAME, "16");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_tooLongField)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_tooLongField));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_tooLongField));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/too_long_field_name", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_tooLongValue(const hmap_t *h)
{
	int res = 0;

	if (++parsing_common.invCnt > 1) {
		fprintf(stderr, "Too many converter invocations");
		parsing_common.corrData = -1;

		return -1;
	}

	res += test_fieldCheck(h, INT1_FIELD_NAME, "8");
	res += test_fieldCheck(h, INT2_FIELD_NAME, "16");

	parsing_common.corrData = res;

	return res;
}


TEST(group_parser_execute, parser_execute_tooLongValue)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_tooLongValue));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_tooLongValue));

	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/too_long_value", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(1, parsing_common.invCnt);
}


static int converter_nullArguments(const hmap_t *h)
{
	fprintf(stderr, "Too many converter invocations");
	parsing_common.invCnt++;
	parsing_common.corrData = -1;

	return -1;
}


TEST(group_parser_execute, parser_execute_nullArguments)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_nullArguments));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_nullArguments));

	/* Pointer to parser is NULL */
	TEST_ASSERT_NOT_EQUAL_INT(0,
		parser_execute(NULL, "usr/test/parser/multipleStructures", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(0, parsing_common.invCnt);

	/* Path to file is NULL */
	TEST_ASSERT_NOT_EQUAL_INT(0, parser_execute(parsing_common.p, NULL, PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(0, parsing_common.invCnt);
}


TEST_GROUP_RUNNER(group_parser_execute)
{
	RUN_TEST_CASE(group_parser_execute, parser_execute_oneHeader);
	RUN_TEST_CASE(group_parser_execute, parser_execute_multipleHeaders);
	RUN_TEST_CASE(group_parser_execute, parser_execute_comments);
	RUN_TEST_CASE(group_parser_execute, parser_execute_spaces);
	RUN_TEST_CASE(group_parser_execute, parser_execute_signs);
	RUN_TEST_CASE(group_parser_execute, parser_execute_ignoreUnknowHeaders);
	RUN_TEST_CASE(group_parser_execute, parser_execute_emptyFile);
	RUN_TEST_CASE(group_parser_execute, parser_execute_unspecifiedHeader);
	RUN_TEST_CASE(group_parser_execute, parser_execute_tooManyFields);
	RUN_TEST_CASE(group_parser_execute, parser_execute_redundantFields);
	RUN_TEST_CASE(group_parser_execute, parser_execute_fieldWithoutHeader1);
	RUN_TEST_CASE(group_parser_execute, parser_execute_fieldWithoutHeader2);
	RUN_TEST_CASE(group_parser_execute, parser_execute_failAtTheBeginning);
	RUN_TEST_CASE(group_parser_execute, parser_execute_failInTheMiddle);
	RUN_TEST_CASE(group_parser_execute, parser_execute_tooLongField);
	RUN_TEST_CASE(group_parser_execute, parser_execute_tooLongValue);
	RUN_TEST_CASE(group_parser_execute, parser_execute_nullArguments);
}


/* ##############################################################################
 * ----------------------        parser_clear tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_clear);


TEST_SETUP(group_parser_clear)
{
	/* Setting up the parser */
	parsing_common.p = parser_alloc(HEADERS_NB, TEST_MAX_FIELDS_NB);
	TEST_ASSERT_NOT_NULL(parsing_common.p);

	parsing_common.corrData = 0;
	parsing_common.invCnt = 0;
}


TEST_TEAR_DOWN(group_parser_clear)
{
	parser_free(parsing_common.p);
}


static int converter_wrong(const hmap_t *h)
{
	fprintf(stderr, "Too many converter invocations");
	parsing_common.invCnt++;
	parsing_common.corrData = -1;

	return -1;
}


TEST(group_parser_clear, parser_clear_std)
{
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_wrong));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_wrong));

	parser_clear(parsing_common.p);

	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT1_HEADER_NAME, converter_multipleHeaders));
	TEST_ASSERT_EQUAL_INT(0, parser_headerAdd(parsing_common.p, TEST_STRUCT2_HEADER_NAME, converter_multipleHeaders));

	TEST_ASSERT_EQUAL_INT(0, parser_execute(parsing_common.p, "usr/test/parser/multiple_headers", PARSER_EXEC_ALL_HEADERS));

	TEST_ASSERT_EQUAL_INT(0, parsing_common.corrData);
	TEST_ASSERT_EQUAL_INT(4, parsing_common.invCnt);
}


TEST_GROUP_RUNNER(group_parser_clear)
{
	RUN_TEST_CASE(group_parser_clear, parser_clear_std);
}
