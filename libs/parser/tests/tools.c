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

#include "tools.h"

#include <string.h>


void* test_converter1(hmap_t *hmap)
{
	int strLen;
	char* string;
	test_struct1_t* result = malloc(sizeof(test_struct1_t));
	if (result == NULL) {
		return NULL;
	}

	result->fieldInt1 = atoi((char*)hmap_get(hmap, INT1_HEADER_NAME));
	result->fieldFloat = atof((char*)hmap_get(hmap, FLOAT_HEADER_NAME));

	string = hmap_get(hmap, STRING_HEADER_NAME);
	strLen = strlen(string);
	result->fieldString = malloc(sizeof(char) * (strLen + 1));
	if (result->fieldString == NULL) {
		free(result);
		return NULL;
	}
	strcpy(result->fieldString, string);

	return result;
}


void* test_converter2(hmap_t *hmap)
{
	test_struct2_t* result = malloc(sizeof(test_struct2_t));
	if (result == NULL) {
		return NULL;
	}

	result->fieldInt1 = atoi((char*)hmap_get(hmap, INT1_HEADER_NAME));
	result->fieldInt2 = atoi((char*)hmap_get(hmap, INT2_HEADER_NAME));

	return result;
}


void* test_failingConverter3(hmap_t *hmap)
{
	static int i = 0;
	test_struct3_t* result;

	i = (i + 1) % (failingRate + 1);

	if ( i == failingRate) {
		return NULL;
	}

	result = malloc(sizeof(test_struct3_t));
	if (result == NULL) {
		return NULL;
	}

	result->fieldInt1 = atoi((char*)hmap_get(hmap, INT1_HEADER_NAME));

	return result;
}


void* null_converter(hmap_t *hmap)
{
    return NULL;
}
