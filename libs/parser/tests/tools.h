/*
 * Phoenix-Pilot
 *
 * Tools for unit tests `parse_alloc` function
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef TOOLS_H
#define TOOLS_H

#include <hmap.h>


#define CHECK_OK 0

#define INT1_HEADER_NAME "INT1"
#define INT2_HEADER_NAME "INT2"
#define FLOAT_HEADER_NAME "FLOAT"
#define STRING_HEADER_NAME "STRING"

#define TEST_STRUCT1_HEADER_NAME "STRUCT1"
#define TEST_STRUCT1_FIELDS_NO 3

#define TEST_ASSERT_STRUCT1(INT1, FLOAT, STRING, structToCheckPtr) \
	TEST_ASSERT_EQUAL_INT((INT1), (structToCheckPtr)->fieldInt1); \
	TEST_ASSERT_EQUAL_FLOAT((FLOAT), (structToCheckPtr)->fieldFloat); \
	TEST_ASSERT_EQUAL_STRING((STRING), (structToCheckPtr)->fieldString);

#define TEST_ASSERT_STRUCT2(INT1, INT2, structToCheckPtr) \
	TEST_ASSERT_EQUAL_INT((INT1), (structToCheckPtr)->fieldInt1); \
	TEST_ASSERT_EQUAL_INT((INT2), (structToCheckPtr)->fieldInt2);

typedef struct
{
	int fieldInt1;
	float fieldFloat;
	char* fieldString;
} test_struct1_t;


#define TEST_STRUCT2_HEADER_NAME "STRUCT2"
#define TEST_STRUCT2_FIELDS_NO 2

typedef struct
{
	int fieldInt1;
    int fieldInt2;
} test_struct2_t;


#define TEST_STRUCT3_HEADER_NAME "STRUCT3"
#define TEST_STRUCT3_FIELDS_NO 1

typedef struct 
{
	int fieldInt1;
} test_struct3_t;



enum test_structs {enumTestStruct1, enumTestStruct2, enumTestStruct3};


extern void* test_converter1(hmap_t *hmap);


extern void* test_converter2(hmap_t *hmap);


extern void* test_failingConverter3(hmap_t *hmap);

static int failingRate = 3;


extern void* null_converter(hmap_t *hmap);


#endif
