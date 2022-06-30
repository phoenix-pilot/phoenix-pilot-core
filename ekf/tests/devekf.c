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

#include "../tools/rotas_dummy.h"


enum printMode { prntVersor, prntAtt };


static void printUavVersors(ekf_state_t *uavState)
{
	vec_t start = { .x = uavState->enuX, .y = uavState->enuY, .z = uavState->enuZ };
	vec_t x = { .x = 1, .y = 0, .z = 0 }, y = { .x = 0, .y = 1, .z = 0 }, z = { .x = 0, .y = 0, .z = 1 };
	quat_t q = { .a = uavState->q0, .i = uavState->q1, .j = uavState->q2, .k = uavState->q3 };

	quat_vecRot(&x, q);
	quat_vecRot(&y, q);
	quat_vecRot(&z, q);

	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z, start->x, start->y, start->z);
}


static void printUavAtt(ekf_state_t *uavState)
{
	printf("YPR: %f %f %f YPR_DOT %f %f %f\n", uavState->yaw,  uavState->pitch,  uavState->roll, uavState->yaw_dot,  uavState->pitch_dot,  uavState->roll_dot);
}


int main(int argc, char **argv)
{
	ekf_state_t uavState;

	vec_t start = { 0 };
	int i = 0;
	enum printMode mode = prntVersor;

	if (argc > 1) {
		if (atoi(argv[1]) == 1) {
			mode = prntAtt;
		}
	}

	if (ekf_init() == 0) {
		ekf_run();
	}

	while (i < 1000) {
		usleep(1000 * 100);
		ekf_stateGet(&uavState);
		if (mode == prntVersor) {
			printUavVersors(&uavState);
		}
		if (mode == prntAtt) {
			printUavAtt(&uavState);
		}
		i++;
	}

	ekf_done();

	return 0;
}
