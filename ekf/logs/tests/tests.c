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

#include <stdio.h>
#include <unistd.h>
#include <errno.h>


#include "../writer.h"
#include "../reader.h"

#include "data.h"
#include "tools.h"

#define EKFLOG_TEST_FILE "tmp/ekf_logs_test.bin"

#define SHORT_SEQUENCE_LEN 10
#define LONG_SEQUENCE_LEN  100


/* Variables for tests */
static time_t timeRead;
static sensor_event_t sensEvt1, sensEvt2, sensEvt3;


TEST_GROUP(group_ekf_logs);


TEST_SETUP(group_ekf_logs)
{
	TEST_ASSERT_EQUAL(0, ekflog_writerInit(EKFLOG_TEST_FILE, EKFLOG_SENSC | EKFLOG_TIME | EKFLOG_STRICT_MODE));
	TEST_ASSERT_EQUAL(0, ekflog_readerInit(EKFLOG_TEST_FILE));

	timeRead = 0;
	ekflogTests_sensorEvtClear(&sensEvt1);
	ekflogTests_sensorEvtClear(&sensEvt2);
	ekflogTests_sensorEvtClear(&sensEvt3);

	errno = 0;
}


TEST_TEAR_DOWN(group_ekf_logs)
{
	if (ekflog_readerDone() != 0) {
		fprintf(stderr, "ekflog tests: error while reader deinit\n");
	}

	if (access(EKFLOG_TEST_FILE, F_OK) == 0) {
		if (remove(EKFLOG_TEST_FILE) != 0) {
			fprintf(stderr, "ekflog tests: cannot remove test file\n");
		}
	}
}


TEST(group_ekf_logs, ekflogs_singleTimeEvt)
{
	TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp1));

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
	TEST_ASSERT_EQUAL(testTimestamp1, timeRead);

	TEST_ASSERT_EQUAL(EOF, ekflog_timeRead(&timeRead));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_multipleTimeEvt)
{
	int i;

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp1));
		TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp2));
	}

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
		TEST_ASSERT_EQUAL(testTimestamp1, timeRead);

		TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
		TEST_ASSERT_EQUAL(testTimestamp2, timeRead);
	}

	TEST_ASSERT_EQUAL(EOF, ekflog_timeRead(&timeRead));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_singleImuEvt)
{
	TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt1, &testGyrEvt1, &testMagEvt1));

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

	TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt1, &sensEvt1));
	TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt1, &sensEvt2));
	TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt1, &sensEvt3));

	TEST_ASSERT_EQUAL(EOF, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_multipleImuEvt)
{
	int i;

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt1, &testGyrEvt1, &testMagEvt1));
		TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt2, &testGyrEvt2, &testMagEvt2));
	}

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt1, &sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt1, &sensEvt2));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt1, &sensEvt3));

		TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt2, &sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt2, &sensEvt2));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt2, &sensEvt3));
	}

	TEST_ASSERT_EQUAL(EOF, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_singleGpsEvt)
{
	TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt1));

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
	TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt1, &sensEvt1));

	TEST_ASSERT_EQUAL(EOF, ekflog_gpsRead(&sensEvt1));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_multipleGpsEvt)
{
	int i;

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt1));
		TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt2));
	}

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt1, &sensEvt1));

		TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt2, &sensEvt1));
	}

	TEST_ASSERT_EQUAL(EOF, ekflog_gpsRead(&sensEvt1));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_singleBaroEvt)
{
	TEST_ASSERT_EQUAL(0, ekflog_baroWrite(&testBaroEvt));

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	TEST_ASSERT_EQUAL(0, ekflog_baroRead(&sensEvt1));
	TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testBaroEvt, &sensEvt1));

	TEST_ASSERT_EQUAL(EOF, ekflog_baroRead(&sensEvt1));
	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_shortSequence)
{
	int i;

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp1));
		TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt1, &testGyrEvt1, &testMagEvt1));
		TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt1));
	}

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
		TEST_ASSERT_EQUAL(testTimestamp1, timeRead);
		timeRead = 0;
	}

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt1, &sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt1, &sensEvt2));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt1, &sensEvt3));

		ekflogTests_sensorEvtClear(&sensEvt1);
		ekflogTests_sensorEvtClear(&sensEvt2);
		ekflogTests_sensorEvtClear(&sensEvt3);
	}

	for (i = 0; i < SHORT_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt1, &sensEvt1));
		ekflogTests_sensorEvtClear(&sensEvt1);
	}

	TEST_ASSERT_EQUAL(EOF, ekflog_timeRead(&timeRead));
	TEST_ASSERT_EQUAL(EOF, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));
	TEST_ASSERT_EQUAL(EOF, ekflog_gpsRead(&sensEvt1));

	TEST_ASSERT_EQUAL(0, errno);
}


TEST(group_ekf_logs, ekflogs_longSequence)
{
	int i;

	for (i = 0; i < LONG_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp1));
		TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt1, &testGyrEvt1, &testMagEvt1));
		TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt1));

		TEST_ASSERT_EQUAL(0, ekflog_timeWrite(testTimestamp2));
		TEST_ASSERT_EQUAL(0, ekflog_imuWrite(&testAccEvt2, &testGyrEvt2, &testMagEvt2));
		TEST_ASSERT_EQUAL(0, ekflog_gpsWrite(&testGpsEvt2));
	}

	TEST_ASSERT_EQUAL(0, ekflog_writerDone());

	for (i = 0; i < LONG_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
		TEST_ASSERT_EQUAL(testTimestamp1, timeRead);

		TEST_ASSERT_EQUAL(0, ekflog_timeRead(&timeRead));
		TEST_ASSERT_EQUAL(testTimestamp2, timeRead);
	}

	for (i = 0; i < LONG_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt1, &sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt1, &sensEvt2));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt1, &sensEvt3));

		TEST_ASSERT_EQUAL(0, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));

		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testAccEvt2, &sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGyrEvt2, &sensEvt2));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testMagEvt2, &sensEvt3));
	}

	for (i = 0; i < LONG_SEQUENCE_LEN; i++) {
		TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt1, &sensEvt1));

		TEST_ASSERT_EQUAL(0, ekflog_gpsRead(&sensEvt1));
		TEST_ASSERT_TRUE(ekflogTests_sensorEvtEqual(&testGpsEvt2, &sensEvt1));
	}

	TEST_ASSERT_EQUAL(EOF, ekflog_timeRead(&timeRead));
	TEST_ASSERT_EQUAL(EOF, ekflog_imuRead(&sensEvt1, &sensEvt2, &sensEvt3));
	TEST_ASSERT_EQUAL(EOF, ekflog_gpsRead(&sensEvt1));

	TEST_ASSERT_EQUAL(0, errno);
}


TEST_GROUP_RUNNER(group_ekf_logs)
{
	RUN_TEST_CASE(group_ekf_logs, ekflogs_singleTimeEvt);
	RUN_TEST_CASE(group_ekf_logs, ekflogs_multipleTimeEvt);

	RUN_TEST_CASE(group_ekf_logs, ekflogs_singleImuEvt);
	RUN_TEST_CASE(group_ekf_logs, ekflogs_multipleImuEvt);

	RUN_TEST_CASE(group_ekf_logs, ekflogs_singleGpsEvt);
	RUN_TEST_CASE(group_ekf_logs, ekflogs_multipleGpsEvt);

	RUN_TEST_CASE(group_ekf_logs, ekflogs_singleBaroEvt);

	RUN_TEST_CASE(group_ekf_logs, ekflogs_shortSequence);
	RUN_TEST_CASE(group_ekf_logs, ekflogs_longSequence);
}
