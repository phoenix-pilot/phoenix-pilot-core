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


enum printMode { prntVersor, prntAtt, printAcc };


static void printUavVersors(ekf_state_t *uavState)
{
	vec_t start = { .x = uavState->enuX, .y = uavState->enuY, .z = uavState->enuZ };
	vec_t x = { .x = 1, .y = 0, .z = 0 }, y = { .x = 0, .y = 1, .z = 0 }, z = { .x = 0, .y = 0, .z = 1 };
	quat_t q = { .a = uavState->q0, .i = uavState->q1, .j = uavState->q2, .k = uavState->q3 };

	quat_vecRot(&x, &q);
	quat_vecRot(&y, &q);
	quat_vecRot(&z, &q);

	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z, start.x, start.y, start.z);
}


static inline void printUavAtt(ekf_state_t *uavState)
{
	printf("YPR: %f %f %f YPR_DOT %f %f %f\n", uavState->yaw * 180 / 3.1415, uavState->pitch * 180 / 3.1415, uavState->roll * 180 / 3.1415, uavState->yawDot, uavState->pitchDot, uavState->rollDot);
}


static inline void printUavAcc(ekf_state_t *uavState)
{
	printf("XYZ %f %f %f\n", uavState->accelX, uavState->accelY, uavState->accelZ);
}


int main(int argc, char **argv)
{
	ekf_state_t uavState;
	enum printMode mode = prntVersor;

	if (argc != 2) {
		printf("Wrong arguments count!\n");
		return EXIT_FAILURE;
	}

	if (atoi(argv[1]) == 1) {
		mode = prntAtt;
	}
	if (atoi(argv[1]) == 2) {
		mode = printAcc;
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
		if (mode == printAcc) {
			printUavAcc(&uavState);
		}
	}

	ekf_done();

	return EXIT_SUCCESS;
}
