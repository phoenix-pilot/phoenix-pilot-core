/*
 * EKF test runner
 *
 * Executes ekf via devekf with adjusted `ekf.conf` file.
 * Compares final ekf state with expected one.
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

#include <hmap.h>

#include "config_file_handler.h"
#include "result_check.h"


#define LOG_FILE "ekf_log.bin"


static int ekftests_testSetUp(const char *scenarioName)
{
	const size_t changes = 4;
	const char *fieldsToChange[changes];
	const size_t scenarioFieldSize = MAX_FILE_TO_CHANGE_STR_LEN + 1;
	char scenarioField[scenarioFieldSize];

	if (strlen(scenarioName) > MAX_VALUE_LEN) {
		fprintf(stderr, "Path to scenario file is too long\n");
		return -1;
	}

	snprintf(scenarioField, scenarioFieldSize, "DATA_SOURCE/file=%s", scenarioName);

	fieldsToChange[0] = "DATA_SOURCE/source=LOGS";
	fieldsToChange[1] = "LOGGING/log=ALL";
	fieldsToChange[2] = "LOGGING/mode=STRICT";
	fieldsToChange[3] = scenarioField;

	return ekftests_configPrepare(fieldsToChange, changes);
}


static int ekftests_testTearDown(void)
{
	return ekftests_restoreConfig();
}


static void ekftests_usage(void)
{
	printf("Usage: ekf_test_runner <ekf_scenario_file> <expected_result_file> [-h]\n\n");
	printf("<ekf_scenario_file> - must lead to valid EKF binary logs, which contains input data for EKF\n");
	printf("<expected_result_file> - must lead to valid file with expected end last EKF status\n\n");
	printf("-h option shows this help info\n");
}


int main(int argc, char **argv)
{
	int opt;
	char *scenarioName;
	char *expectedResult;

	while ((opt = getopt(argc, argv, "h")) != -1) {
		switch (opt) {
			case 'h':
				ekftests_usage();
				return EXIT_SUCCESS;

			case '?':
				fprintf(stderr, "Unknown option %c\n\n", optopt);
				ekftests_usage();
				return EXIT_FAILURE;
		}
	}

	if (argc != 3) {
		fprintf(stderr, "Invalid program usage\n\n");
		ekftests_usage();
		return EXIT_FAILURE;
	}

	scenarioName = argv[1];
	expectedResult = argv[2];

	if (ekftests_testSetUp(scenarioName) != 0) {
		fprintf(stderr, "\033[31m\nTEST FAILED\033[39m: Error occurred during test set up\n");
		return EXIT_FAILURE;
	}

	if (system("usr/bin/devekf 1") != 0) {
		fprintf(stderr, "\033[31m\nTEST FAILED\033[39m: Error occurred during ekf execution\n");
		ekftests_testTearDown();

		/* Ekf error is not an error of a test runner */
		return EXIT_SUCCESS;
	}

	printf("\033[34m\nChecking output\n\n\033[39m");

	if (ekftests_resultCheck(LOG_FILE, expectedResult) == 0) {
		printf("\n\033[32mTEST PASSED\n\033[39m");
	}
	else {
		printf("\033[31m\nTEST FAILED\n\033[39m");
	}

	ekftests_testTearDown();

	return EXIT_SUCCESS;
}
