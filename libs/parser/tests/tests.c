/*
 * Phoenix-Pilot
 *
 * Unit tests `parse_alloc` function
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

#include "tools.h"

#include <stdlib.h>


#define bufLen 5


static parser_t* parser;
static parser_result_t* result;
static int resultLen;

/* Must be at least 2 */
static const int headers = 5;


/* ##############################################################################
 * ----------------------        parser_alloc tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_alloc);

TEST_SETUP(group_parser_alloc)
{
	parser = NULL;
}


TEST_TEAR_DOWN(group_parser_alloc)
{
    parser_free(parser);
}


TEST(group_parser_alloc, parser_alloc_std)
{
    parser = parser_alloc(headers);

    TEST_ASSERT_NOT_NULL(parser);
}


TEST(group_parser_alloc, parser_alloc_neqHeaderNo)
{
	parser = parser_alloc(-headers);

	TEST_ASSERT_NULL(parser);
}


TEST(group_parser_alloc, parser_alloc_zeroHeaders)
{
	parser = parser_alloc(0);

	TEST_ASSERT_NULL(parser);
}


TEST(group_parser_alloc, parser_alloc_tooManyHeaders)
{
	parser = parser_alloc(INT_MAX);

	TEST_ASSERT_NULL(parser);
}


TEST_GROUP_RUNNER(group_parser_alloc)
{
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_std);
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_neqHeaderNo);
	RUN_TEST_CASE(group_parser_alloc, parser_alloc_zeroHeaders);
	/*RUN_TEST_CASE(group_parser_alloc, parser_alloc_tooManyHeaders);*/
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
	parser = NULL;
	parser_free(parser);

	TEST_ASSERT_NULL(parser);
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
	parser = parser_alloc(headers);
	TEST_ASSERT_NOT_NULL(parser);
}


TEST_TEAR_DOWN(group_parser_headerAdd)
{
	parser_free(parser);
}


TEST(group_parser_headerAdd, parser_headerAdd_oneHeader)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
}


TEST(group_parser_headerAdd, parser_headerAdd_multipleHeader)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT2_HEADER_NAME, TEST_STRUCT2_FIELDS_NO, enumTestStruct2, test_converter2));
}


TEST(group_parser_headerAdd, parser_headerAdd_passingNull)
{
	/* Parser as NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(NULL, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));

	/* Header name as NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, NULL, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));

	/*  Converter as NULL*/
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, NULL));
}


TEST(group_parser_headerAdd, parser_headerAdd_tooManyHeader)
{
	int i;
	char buf[bufLen];

	for (i = 0; i < headers; i++) {
		snprintf(buf, bufLen, "%d", i);

		TEST_ASSERT_EQUAL_INT(CHECK_OK,
			parser_headerAdd(parser, buf, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, null_converter));
	}

	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT2_HEADER_NAME, TEST_STRUCT2_FIELDS_NO, enumTestStruct2, test_converter2));
}


TEST(group_parser_headerAdd, parser_headerAdd_addingHeaderTwoTimes)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
}


TEST(group_parser_headerAdd, parser_headerAdd_zeroFields)
{
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, 0, enumTestStruct1, test_converter1));
}


TEST_GROUP_RUNNER(group_parser_headerAdd)
{
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_oneHeader);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_multipleHeader);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_passingNull);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_tooManyHeader);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_addingHeaderTwoTimes);
	RUN_TEST_CASE(group_parser_headerAdd, parser_headerAdd_zeroFields);
}


/* ##############################################################################
 * ----------------------        parser_parse tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_parse);


TEST_SETUP(group_parser_parse)
{
	parser = parser_alloc(headers);
	TEST_ASSERT_NOT_NULL(parser);

	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT2_HEADER_NAME, TEST_STRUCT2_FIELDS_NO, enumTestStruct2, test_converter2));

	result = NULL;
	resultLen = -headers;
}


TEST_TEAR_DOWN(group_parser_parse)
{
	int i;
	
	if (result != NULL) {
		for (i=0; i< resultLen; i++) {
			if(result[i].structFlag == enumTestStruct1) {
				free(((test_struct1_t*)result[i].data)->fieldString);
			}

			free(result[i].data);
		}

		free(result);
	}
	
	parser_free(parser);
}


TEST(group_parser_parse, parser_parse_oneStructure)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/oneStructure", &result, &resultLen ));

	TEST_ASSERT_NOT_NULL(result);
	TEST_ASSERT_EQUAL_INT(1, resultLen);

	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[0].structFlag);
	TEST_ASSERT_NOT_NULL(result[0].data);
	TEST_ASSERT_STRUCT1(5, 2.5f, "TEST", (test_struct1_t*)(result[0].data));
}


TEST(group_parser_parse, parser_parse_multipleStructures)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/multipleStructures", &result, &resultLen));
	
	TEST_ASSERT_NOT_NULL(result);
	TEST_ASSERT_EQUAL_INT(4, resultLen);

	/* First structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[0].structFlag);
	TEST_ASSERT_NOT_NULL(result[0].data);
	TEST_ASSERT_STRUCT1(5, 2.5f, "TEST", (test_struct1_t*)(result[0].data));

	/* Second structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[1].structFlag);
	TEST_ASSERT_NOT_NULL(result[1].data);
	TEST_ASSERT_STRUCT2(8, 16, (test_struct2_t*)(result[1].data));

	/* Third structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[2].structFlag);
	TEST_ASSERT_NOT_NULL(result[2].data);
	TEST_ASSERT_STRUCT1(27, 56.25f, "STR", (test_struct1_t*)(result[2].data));

	/* Fourth structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[3].structFlag);
	TEST_ASSERT_NOT_NULL(result[3].data);
	TEST_ASSERT_STRUCT2(-25, 123, (test_struct2_t*)(result[3].data));
}


TEST(group_parser_parse, parser_parse_comments)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/comments", &result, &resultLen));
	
	TEST_ASSERT_NOT_NULL(result);
	TEST_ASSERT_EQUAL_INT(2, resultLen);

	/* First structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[0].structFlag);
	TEST_ASSERT_NOT_NULL(result[0].data);
	TEST_ASSERT_STRUCT1(5, 2.5f, "TEST", (test_struct1_t*)(result[0].data));

	/* Second structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[1].structFlag);
	TEST_ASSERT_NOT_NULL(result[1].data);
	TEST_ASSERT_STRUCT2(8, 16, (test_struct2_t*)(result[1].data));
}


TEST(group_parser_parse, parser_parse_spaces)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/spaces", &result, &resultLen));
	
	TEST_ASSERT_NOT_NULL(result);
	TEST_ASSERT_EQUAL_INT(2, resultLen);

	/* First structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[0].structFlag);
	TEST_ASSERT_NOT_NULL(result[0].data);
	TEST_ASSERT_STRUCT1(5, 2.5f, "TEST", (test_struct1_t*)(result[0].data));

	/* Second structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[1].structFlag);
	TEST_ASSERT_NOT_NULL(result[1].data);
	TEST_ASSERT_STRUCT2(8, 16, (test_struct2_t*)(result[1].data));
}


TEST(group_parser_parse, parser_parse_emptyFile)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/emptyFile", &result, &resultLen));
	
	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_unspecifiedHeader)
{
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/unspecifiedHeader", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_notEnoughFields)
{
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/notEnoughFields", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_tooManyFields)
{
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/tooManyFields", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_redundantFields)
{
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/redundantFields", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_converterFailsAtBeginning)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT3_HEADER_NAME, TEST_STRUCT3_FIELDS_NO, enumTestStruct3, null_converter));
	
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/redundantFields", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);
}


TEST(group_parser_parse, parser_parse_converterFailsInTheMiddle)
{
	int i;
	
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT3_HEADER_NAME, TEST_STRUCT3_FIELDS_NO, enumTestStruct3, test_failingConverter3));
	
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/failingInTheMiddle", &result, &resultLen));

	TEST_ASSERT_EQUAL_INT(failingRate - 1, resultLen);

	for (i=0; i< resultLen; i++) {
		TEST_ASSERT_NOT_NULL(result[i].data);
		TEST_ASSERT_EQUAL_INT(enumTestStruct3, result[i].structFlag);

		TEST_ASSERT_EQUAL_INT(i+1, ((test_struct3_t*)result[i].data)->fieldInt1);
	}
}


TEST(group_parser_parse, parser_parse_passingNullArguments)
{
	/* Pointer to parser is NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(NULL, "usr/test/parser/multipleStructures", &result, &resultLen));
	
	/* Pointer to result is NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/multipleStructures", NULL, &resultLen));

	TEST_ASSERT_EQUAL_INT(0, resultLen);

	/* Pointer to length of the result is NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/multipleStructures", &result, NULL));

	/* All of these pointes are NULL */
	TEST_ASSERT_NOT_EQUAL_INT(CHECK_OK,
		parser_parse(NULL, "usr/test/parser/multipleStructures", NULL, NULL));
}


TEST_GROUP_RUNNER(group_parser_parse)
{
	RUN_TEST_CASE(group_parser_parse, parser_parse_oneStructure);
	RUN_TEST_CASE(group_parser_parse, parser_parse_multipleStructures);
	RUN_TEST_CASE(group_parser_parse, parser_parse_comments);
	RUN_TEST_CASE(group_parser_parse, parser_parse_spaces);
	RUN_TEST_CASE(group_parser_parse, parser_parse_emptyFile);
	RUN_TEST_CASE(group_parser_parse, parser_parse_unspecifiedHeader);
	RUN_TEST_CASE(group_parser_parse, parser_parse_notEnoughFields);
	RUN_TEST_CASE(group_parser_parse, parser_parse_tooManyFields);
	RUN_TEST_CASE(group_parser_parse, parser_parse_redundantFields);
	RUN_TEST_CASE(group_parser_parse, parser_parse_converterFailsAtBeginning);
	RUN_TEST_CASE(group_parser_parse, parser_parse_converterFailsInTheMiddle);
	RUN_TEST_CASE(group_parser_parse, parser_parse_passingNullArguments);
}


/* ##############################################################################
 * ----------------------        parser_clear tests       -----------------------
 * ############################################################################## */


TEST_GROUP(group_parser_clear);


TEST_SETUP(group_parser_clear)
{
	parser = parser_alloc(headers);
	TEST_ASSERT_NOT_NULL(parser);

	result = NULL;
	resultLen = -headers;
}


TEST_TEAR_DOWN(group_parser_clear)
{
	int i;
	
	if (result != NULL) {
		for (i=0; i< resultLen; i++) {
			if(result[i].structFlag == enumTestStruct1) {
				free(((test_struct1_t*)result[i].data)->fieldString);
			}

			free(result[i].data);
		}

		free(result);
	}
	
	parser_free(parser);
}


TEST(group_parser_clear, parser_clear_std)
{
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, null_converter));
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT2_HEADER_NAME, TEST_STRUCT2_FIELDS_NO, enumTestStruct2, null_converter));

	parser_clear(parser);

	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT1_HEADER_NAME, TEST_STRUCT1_FIELDS_NO, enumTestStruct1, test_converter1));
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_headerAdd(parser, TEST_STRUCT2_HEADER_NAME, TEST_STRUCT2_FIELDS_NO, enumTestStruct2, test_converter2));
	
	TEST_ASSERT_EQUAL_INT(CHECK_OK,
		parser_parse(parser, "usr/test/parser/multipleStructures", &result, &resultLen));
	
	TEST_ASSERT_NOT_NULL(result);
	TEST_ASSERT_EQUAL_INT(4, resultLen);

	/* First structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[0].structFlag);
	TEST_ASSERT_NOT_NULL(result[0].data);
	TEST_ASSERT_STRUCT1(5, 2.5f, "TEST", (test_struct1_t*)(result[0].data));

	/* Second structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[1].structFlag);
	TEST_ASSERT_NOT_NULL(result[1].data);
	TEST_ASSERT_STRUCT2(8, 16, (test_struct2_t*)(result[1].data));

	/* Third structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct1, result[2].structFlag);
	TEST_ASSERT_NOT_NULL(result[2].data);
	TEST_ASSERT_STRUCT1(27, 56.25f, "STR", (test_struct1_t*)(result[2].data));

	/* Fourth structure */
	TEST_ASSERT_EQUAL_INT(enumTestStruct2, result[3].structFlag);
	TEST_ASSERT_NOT_NULL(result[3].data);
	TEST_ASSERT_STRUCT2(-25, 123, (test_struct2_t*)(result[3].data));
}


TEST(group_parser_clear, parser_clear_passingNull)
{
	parser_clear(NULL);
}


TEST_GROUP_RUNNER(group_parser_clear)
{
	RUN_TEST_CASE(group_parser_clear, parser_clear_std);
	RUN_TEST_CASE(group_parser_clear, parser_clear_passingNull);
}
