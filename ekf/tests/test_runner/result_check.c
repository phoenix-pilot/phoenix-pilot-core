/*
 * Phoenix-Pilot
 *
 * EKF test runner result checking routines
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "result_check.h"

#include "../../kalman_implem.h"
#include "../../logs/reader.h"

#include <matrix.h>
#include <hmap.h>
#include <parser.h>

#include <errno.h>
#include <stdbool.h>
#include <math.h>


#define EPS_FIELDS_NB 5


static struct {
	matrix_t *expectedState;
	matrix_t *eps;
} ekfTestsResult_common;


static int ekftests_expectedResultConverter(const hmap_t *h)
{
	int err = 0;
	float eps;

	err |= parser_fieldGetFloat(h, "attitude_a", &ekfTestsResult_common.expectedState->data[QA]);
	err |= parser_fieldGetFloat(h, "attitude_i", &ekfTestsResult_common.expectedState->data[QB]);
	err |= parser_fieldGetFloat(h, "attitude_j", &ekfTestsResult_common.expectedState->data[QC]);
	err |= parser_fieldGetFloat(h, "attitude_k", &ekfTestsResult_common.expectedState->data[QD]);

	err |= parser_fieldGetFloat(h, "gyro_bias_x", &ekfTestsResult_common.expectedState->data[BWX]);
	err |= parser_fieldGetFloat(h, "gyro_bias_y", &ekfTestsResult_common.expectedState->data[BWY]);
	err |= parser_fieldGetFloat(h, "gyro_bias_z", &ekfTestsResult_common.expectedState->data[BWZ]);

	err |= parser_fieldGetFloat(h, "velocity_x", &ekfTestsResult_common.expectedState->data[VX]);
	err |= parser_fieldGetFloat(h, "velocity_y", &ekfTestsResult_common.expectedState->data[VY]);
	err |= parser_fieldGetFloat(h, "velocity_z", &ekfTestsResult_common.expectedState->data[VZ]);

	err |= parser_fieldGetFloat(h, "accel_bias_x", &ekfTestsResult_common.expectedState->data[BAX]);
	err |= parser_fieldGetFloat(h, "accel_bias_y", &ekfTestsResult_common.expectedState->data[BAY]);
	err |= parser_fieldGetFloat(h, "accel_bias_z", &ekfTestsResult_common.expectedState->data[BAZ]);

	err |= parser_fieldGetFloat(h, "position_x", &ekfTestsResult_common.expectedState->data[RX]);
	err |= parser_fieldGetFloat(h, "position_y", &ekfTestsResult_common.expectedState->data[RY]);
	err |= parser_fieldGetFloat(h, "position_z", &ekfTestsResult_common.expectedState->data[RZ]);

	err |= parser_fieldGetFloat(h, "attitude_eps", &eps);
	ekfTestsResult_common.eps->data[QA] = eps;
	ekfTestsResult_common.eps->data[QB] = eps;
	ekfTestsResult_common.eps->data[QC] = eps;
	ekfTestsResult_common.eps->data[QD] = eps;

	err |= parser_fieldGetFloat(h, "gyro_bias_eps", &eps);
	ekfTestsResult_common.eps->data[BWX] = eps;
	ekfTestsResult_common.eps->data[BWY] = eps;
	ekfTestsResult_common.eps->data[BWZ] = eps;

	err |= parser_fieldGetFloat(h, "velocity_eps", &eps);
	ekfTestsResult_common.eps->data[VX] = eps;
	ekfTestsResult_common.eps->data[VY] = eps;
	ekfTestsResult_common.eps->data[VZ] = eps;

	err |= parser_fieldGetFloat(h, "accel_bias_eps", &eps);
	ekfTestsResult_common.eps->data[BAX] = eps;
	ekfTestsResult_common.eps->data[BAY] = eps;
	ekfTestsResult_common.eps->data[BAZ] = eps;

	err |= parser_fieldGetFloat(h, "position_eps", &eps);
	ekfTestsResult_common.eps->data[RX] = eps;
	ekfTestsResult_common.eps->data[RY] = eps;
	ekfTestsResult_common.eps->data[RZ] = eps;

	return (err == 0) ? 0 : -1;
}


static int ekftests_expectedResultGet(const char *expectedResultFile, matrix_t *expectedState, matrix_t *eps)
{
	parser_t *parser;
	int res;

	ekfTestsResult_common.expectedState = expectedState;
	ekfTestsResult_common.eps = eps;

	parser = parser_alloc(1, STATE_LENGTH + EPS_FIELDS_NB);
	if (parser == NULL) {
		return -1;
	}

	if (parser_headerAdd(parser, "EXP_EKF_STATUS", ekftests_expectedResultConverter) != 0) {
		parser_free(parser);
		return -1;
	}

	res = parser_execute(parser, expectedResultFile, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(parser);

	return res;
}


static int ekftests_finalStateGet(const char *logFile, matrix_t *finalState)
{
	/* Timestamp is needed only to pass valid pointer to `ekflog_stateRead`. Currently we are not checking timestamps */
	time_t timestamp;

	if (ekflog_readerInit(logFile) != 0) {
		return -1;
	}

	errno = 0;

	/* Getting the last ekf state */
	while (ekflog_stateRead(finalState, &timestamp) == 0) { }
	if (ekflog_readerDone() != 0) {
		return -1;
	}

	/* `ekflog_stateRead` sets errno in case of error */
	return errno;
}


static int ekftests_statesAlloc(matrix_t *expected, matrix_t *final, matrix_t *epsilon, matrix_t *diff)
{
	if (matrix_bufAlloc(expected, STATE_LENGTH, 1) != 0) {
		return -1;
	}

	if (matrix_bufAlloc(final, STATE_LENGTH, 1) != 0) {
		matrix_bufFree(expected);
		return -1;
	}

	if (matrix_bufAlloc(epsilon, STATE_LENGTH, 1) != 0) {
		matrix_bufFree(expected);
		matrix_bufFree(final);
		return -1;
	}

	if (matrix_bufAlloc(diff, STATE_LENGTH, 1) != 0) {
		matrix_bufFree(expected);
		matrix_bufFree(final);
		matrix_bufFree(epsilon);
		return -1;
	}

	return 0;
}


static void ekftests_statesFree(matrix_t *expected, matrix_t *final, matrix_t *epsilon, matrix_t *diff)
{
	matrix_bufFree(expected);
	matrix_bufFree(final);
	matrix_bufFree(epsilon);
	matrix_bufFree(diff);
}


static int ekftests_errorPrint(const matrix_t *finalState, const matrix_t *expectedState, const matrix_t *eps, int index)
{
	float final, exp;

	printf("Invalid ");

	switch (index) {
		case QA:
			printf("attitude quaternion real part ");
			break;

		case QB:
			printf("attitude quaternion i part ");
			break;

		case QC:
			printf("attitude quaternion j part ");
			break;

		case QD:
			printf("attitude quaternion k part ");
			break;

		case BWX:
			printf("gyroscope x axis bias ");
			break;

		case BWY:
			printf("gyroscope y axis bias ");
			break;

		case BWZ:
			printf("gyroscope z axis bias ");
			break;

		case VX:
			printf("velocity x component ");
			break;

		case VY:
			printf("velocity y component ");
			break;

		case VZ:
			printf("velocity z component ");
			break;

		case BAX:
			printf("accelerometer x axis bias ");
			break;

		case BAY:
			printf("accelerometer y axis bias ");
			break;

		case BAZ:
			printf("accelerometer z axis bias ");
			break;

		case RX:
			printf("position x component ");
			break;

		case RY:
			printf("position y component ");
			break;

		case RZ:
			printf("position z component ");
			break;

		default:
			printf(" - unknown index\n");
			return -1;
	}

	exp = expectedState->data[index];
	final = finalState->data[index];

	printf("expected: %f, actual: %f, allowed diff: %f, actual diff: %f\n",
		exp, final, eps->data[index], fabs(final - exp));

	return 0;
}


int ekftests_resultCheck(const char *logFile, const char *expectedResultFile)
{
	int i;
	bool testPassed = true;
	matrix_t finalState, expectedState, eps, stateDiff;

	if (ekftests_statesAlloc(&finalState, &expectedState, &eps, &stateDiff) != 0) {
		fprintf(stderr, "Error while allocating ekf statuses\n");
		return -1;
	}

	if (ekftests_expectedResultGet(expectedResultFile, &expectedState, &eps) != 0) {
		fprintf(stderr, "Error while parsing expected result\n");
		ekftests_statesFree(&finalState, &expectedState, &eps, &stateDiff);
		return -1;
	}

	/* Checking if eps matrix is valid */
	for (i = 0; i < STATE_LENGTH; i++) {
		if (eps.data[i] < 0) {
			fprintf(stderr, "Invalid expected result - eps cannot be negative\n");
			ekftests_statesFree(&finalState, &expectedState, &eps, &stateDiff);
			return -1;
		}
	}

	if (ekftests_finalStateGet(logFile, &finalState) != 0) {
		fprintf(stderr, "Error while parsing final ekf status\n");
		ekftests_statesFree(&finalState, &expectedState, &eps, &stateDiff);
		return -1;
	}

	matrix_sub(&finalState, &expectedState, &stateDiff);

	for (i = 0; i < STATE_LENGTH; i++) {
		if (stateDiff.data[i] < -eps.data[i] || stateDiff.data[i] > eps.data[i]) {
			ekftests_errorPrint(&finalState, &expectedState, &eps, i);
			testPassed = false;
		}
	}

	ekftests_statesFree(&finalState, &expectedState, &eps, &stateDiff);

	return testPassed ? 0 : -1;
}
