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

#include <ekflib.h>

#include "../tools/rotas_dummy.h"

static void printUavVersors(quat_t *q, vec_t *start)
{
	vec_t x = { .x = 1, .y = 0, .z = 0 }, y = { .x = 0, .y = 1, .z = 0 }, z = { .x = 0, .y = 0, .z = 1 };

	quat_vecRot(&x, q);
	quat_vecRot(&y, q);
	quat_vecRot(&z, q);

	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z, start->x, start->y, start->z);
}

int main(int argc, char **argv)
{
	ekf_state_t uavState;
	quat_t q;
	vec_t start = { 0 };
	int i = 0;

	if (ekf_init() == 0) {
		ekf_run();
	}

	while (i < 1000) {
		usleep(1000 * 100);
		ekf_stateGet(&uavState);
		q = (quat_t) { .a = uavState.q0, .i = uavState.q1, .j = uavState.q2, .k = uavState.q3 };
		start = (vec_t) { .x = uavState.enuX, .y = uavState.enuY, .z = uavState.enuZ };
		printUavVersors(&q, &start);
		i++;
	}

	ekf_done();

	return 0;
}
