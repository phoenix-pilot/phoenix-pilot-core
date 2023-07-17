/*
 * Phoenix-Pilot
 *
 * Unit tests for ekf log module
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <unity_fixture.h>

#include "../../log.h"

#define EKFLOG_TEST_FILE     "ekf_log_test.txt"
#define EKFLOG_TEST_DATA_LEN 5


struct {
	FILE *testFile;
	char *lineBuf;
	size_t bufLen;
} ekflog_test_common;


static char *ekflog_test_testData[EKFLOG_TEST_DATA_LEN] = {
	"Lorem ipsum\n",
	"dolor sit amet,\n",
	"consectetur adipiscing elit,\n",
	"sed do eiusmod tempor incididunt\n",
	"ut labore et dolore magna aliqua.\n"
};

static char maxLenMsg[MAX_MSG_LEN + 1];


__attribute__((constructor(101))) static void ekflog_test_maxLenMsgFill(void)
{
	memset(maxLenMsg, 'a', MAX_MSG_LEN - 1);
	maxLenMsg[MAX_MSG_LEN - 1] = '\n';
	maxLenMsg[MAX_MSG_LEN] = '\0';
}


static int ekflog_test_fileEnd(void)
{
	return fgetc(ekflog_test_common.testFile) != EOF;
}


static int ekflog_test_fileLineCheck(const char *expectedLine)
{
	if (ekflog_test_common.testFile == NULL) {
		ekflog_test_common.testFile = fopen(EKFLOG_TEST_FILE, "r");
		if (ekflog_test_common.testFile == NULL) {
			fprintf(stderr, "ekflog tests: cannot open test file\n");
			return -1;
		}
	}

	if (getline(&ekflog_test_common.lineBuf, &ekflog_test_common.bufLen, ekflog_test_common.testFile) < 0) {
		if (ekflog_test_fileEnd() == 0) {
			fprintf(stderr, "ekflog test: unexpected EOF encountered\n");
		}
		else {
			fprintf(stderr, "ekflog tests: unknown error while reading file\n");
		}

		return -1;
	}

	if (strcmp(ekflog_test_common.lineBuf, expectedLine) != 0) {
		fprintf(stderr, "ekflog tests: invalid line in file.\n");
		fprintf(stderr, "Expected: %s\n", expectedLine);
		fprintf(stderr, "Actual:   %s\n", ekflog_test_common.lineBuf);
		return -1;
	}

	return 0;
}


TEST_GROUP(group_log);


TEST_SETUP(group_log)
{
	ekflog_test_common.testFile = NULL;
	ekflog_test_common.lineBuf = NULL;
	ekflog_test_common.bufLen = 0;

	TEST_ASSERT_EQUAL(0, ekflog_init(EKFLOG_TEST_FILE, EKFLOG_SENSC | EKFLOG_STRICT_MODE));
}


TEST_TEAR_DOWN(group_log)
{
	if (ekflog_test_common.testFile != NULL) {
		if (fclose(ekflog_test_common.testFile) != 0) {
			fprintf(stderr, "ekflog tests: cannot close test file\n");
		}
	}

	free(ekflog_test_common.lineBuf);

	if (access(EKFLOG_TEST_FILE, F_OK) == 0) {
		if (remove(EKFLOG_TEST_FILE) != 0) {
			fprintf(stderr, "ekflog tests: cannot remove test file\n");
		}
	}
}


TEST(group_log, ekflog_simpleWrite)
{
	TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, ekflog_test_testData[0]));
	TEST_ASSERT_EQUAL(0, ekflog_done());

	TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(ekflog_test_testData[0]));
	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST(group_log, ekflog_singleMaxLenWrite)
{
	TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, maxLenMsg));
	TEST_ASSERT_EQUAL(0, ekflog_done());

	TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(maxLenMsg));
	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST(group_log, ekflog_multipleWrites)
{
	int i;

	for (i = 0; i < EKFLOG_TEST_DATA_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, ekflog_test_testData[i]));
	}

	TEST_ASSERT_EQUAL(0, ekflog_done());

	for (i = 0; i < EKFLOG_TEST_DATA_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(ekflog_test_testData[i]));
	}

	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST(group_log, ekflog_flagsWorkingCheck)
{
	int i;

	for (i = 0; i < EKFLOG_TEST_DATA_LEN / 2; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, ekflog_test_testData[i]));
	}

	TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_MEAS, "This should not be logged\n"));

	for (i = EKFLOG_TEST_DATA_LEN / 2; i < EKFLOG_TEST_DATA_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, ekflog_test_testData[i]));
	}

	TEST_ASSERT_EQUAL(0, ekflog_done());

	for (i = 0; i < EKFLOG_TEST_DATA_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(ekflog_test_testData[i]));
	}

	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST(group_log, ekflog_stressTest)
{
	int i, j;
	const int repeats = 100;

	for (i = 0; i < repeats; i++) {
		for (j = 0; j < EKFLOG_TEST_DATA_LEN; j++) {
			TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, ekflog_test_testData[j]));
		}
	}

	TEST_ASSERT_EQUAL(0, ekflog_done());

	for (i = 0; i < repeats; i++) {
		for (j = 0; j < EKFLOG_TEST_DATA_LEN; j++) {
			TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(ekflog_test_testData[j]));
		}
	}

	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST(group_log, ekflog_multipleMaxLenMsgWrites)
{
	int i;
	const int repeats = 100;

	for (i = 0; i < repeats; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_write(EKFLOG_SENSC, maxLenMsg));
	}

	TEST_ASSERT_EQUAL(0, ekflog_done());

	for (i = 0; i < repeats; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_test_fileLineCheck(maxLenMsg));
	}

	TEST_ASSERT_EQUAL(0, ekflog_test_fileEnd());
}


TEST_GROUP_RUNNER(group_log)
{
	RUN_TEST_CASE(group_log, ekflog_simpleWrite);
	RUN_TEST_CASE(group_log, ekflog_singleMaxLenWrite);
	RUN_TEST_CASE(group_log, ekflog_multipleWrites);
	RUN_TEST_CASE(group_log, ekflog_flagsWorkingCheck);
	RUN_TEST_CASE(group_log, ekflog_stressTest);
	RUN_TEST_CASE(group_log, ekflog_multipleMaxLenMsgWrites);
}