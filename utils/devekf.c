/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * ekf simple client
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <ekflib.h>

#include <vec.h>
#include <quat.h>


enum printMode { prntVersor, prntAtt, printPos, silent };

/* Printing versors of local NED frame of reference in global ENU frame of reference */
static void printUavVersors(ekf_state_t *uavState)
{
	const quat_t ned2enu = { .a = 0, .i = -0.7071f, .j = -0.7071f, .k = 0 };

	vec_t pos = { .x = uavState->enuX, .y = uavState->enuY, .z = uavState->enuZ };
	vec_t versx = { .x = 1, .y = 0, .z = 0 }, versy = { .x = 0, .y = 1, .z = 0 }, versz = { .x = 0, .y = 0, .z = 1 };
	quat_t q = { .a = uavState->q0, .i = uavState->q1, .j = uavState->q2, .k = uavState->q3 };

	/* Rotate versors from local to global frame of reference (all done in NED frame of reference) */
	quat_vecRot(&versx, &q);
	quat_vecRot(&versy, &q);
	quat_vecRot(&versz, &q);

	/* Rotate versors from NED to ENU frame of reference */
	quat_vecRot(&versx, &ned2enu);
	quat_vecRot(&versy, &ned2enu);
	quat_vecRot(&versz, &ned2enu);

	/* Print NED versors in ENU frame of reference */
	printf(
		"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f ",
		versx.x, versx.y, versx.z, versy.x, versy.y, versy.z, versz.x, versz.y, versz.z);

	/* Position is already in ENU, just prinitng it */
	printf("%.3f %.3f %.3f ", pos.x, pos.y, pos.z);
	printf("\n");
}


static inline void printUavAtt(ekf_state_t *uavState)
{
	printf("YPR: %.2f %.2f %.3f YPR_DOT %.3f %.3f %.3f NED/DOT: %.2f %.3f\n", uavState->yaw * 180 / 3.1415, uavState->pitch * 180 / 3.1415, uavState->roll * 180 / 3.1415, uavState->yawDot, uavState->pitchDot, uavState->rollDot, uavState->enuZ, uavState->veloZ);
}


static inline void printUavPos(ekf_state_t *uavState)
{
	printf("A %.0f %.1f %.1f ", uavState->yaw * 180 / 3.1415, uavState->pitch * 180 / 3.1415, uavState->roll * 180 / 3.1415);
	printf("R %.1f %.2f %.2f ", uavState->yawDot, uavState->pitchDot, uavState->rollDot);
	printf("P %.2f %.2f %.2f ", uavState->enuX, uavState->enuY, uavState->enuZ);
	printf("V %.2f %.2f %.2f ", uavState->veloX, uavState->veloY, uavState->veloZ);
	printf("\n");
}


int main(int argc, char **argv)
{
	ekf_state_t uavState;
	enum printMode mode;

	if (argc != 2) {
		printf("Wrong arguments count!\n");
		return EXIT_FAILURE;
	}

	switch (argv[1][0]) {
		case '1':
			mode = prntAtt;
			break;
		case '2':
			mode = printPos;
			break;
		case '3':
			mode = silent;
			break;
		default:
			mode = prntVersor;
			break;
	}

	if (ekf_init() == 0) {
		ekf_run();
	}

	/* This is testing app that may be used to present EKF capabilities and stability, thus no run time is set */
	while (1) {
		usleep(1000 * 100);
		ekf_stateGet(&uavState);
		if (mode == prntVersor) {
			printUavVersors(&uavState);
		}
		if (mode == prntAtt) {
			printUavAtt(&uavState);
		}
		if (mode == printPos) {
			printUavPos(&uavState);
		}
		/* Mode = silent do not invoke any printing function */
	}

	ekf_done();

	return EXIT_SUCCESS;
}
