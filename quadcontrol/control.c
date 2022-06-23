/*
 * Phoenix-RTOS
 *
 * Quadcopter Flight Controller
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-PILOT
 *
 * %LICENSE%
 */

#include "control.h"
#include "mma.h"
#include "pid.h"

#include <ekflib.h>
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include <unistd.h>
#include <sys/time.h>

#define DELTA(measurement, setVal) (fabs((setVal) - (measurement)))
#define ALTITUDE_TOLERANCE         500 /* 1E-3 [m] (millimetres) */

#define PATH_PIDS_CONFIG "/etc/quad.conf"

/* Flag enable hackish code for initial tests which ignore altitude and yaw */
#define TEST_ATTITUDE 1


struct {
	struct {
		pid_ctx_t alt;
		pid_ctx_t roll;
		pid_ctx_t pitch;
		pid_ctx_t yaw;
	} pids;

	time_t lastTime;
} quad_common;


/* Example of flight scenario.
   TODO: move data to the configuration file placed in a rootfs */
#if TEST_ATTITUDE
static const flight_mode_t scenario[] = {
	{ .type = flight_hover, .hover = { .alt = 2000, .time = 40000 } },
	{ .type = flight_end },
};
#else
static const flight_mode_t scenario[] = {
	{ .type = flight_takeoff, .takeoff = { .alt = 4000 } },
	{ .type = flight_hover, .hover = { .alt = 9000, .time = 4000 } },
	{ .type = flight_hover, .hover = { .alt = 12000, .time = 3000 } },
	{ .type = flight_hover, .hover = { .alt = 8000, .time = 3000 } },
	{ .type = flight_hover, .hover = { .alt = 12000, .time = 3000 } },
	{ .type = flight_hover, .hover = { .alt = 8000, .time = 3000 } },
	{ .type = flight_landing },
};
#endif


/* TODO: verify coefficients with test quadcopter */
static const quad_coeffs_t quadCoeffs = {
	.dragCoeff = 7.5E-7,
	.trustCoeff = 3.13E-5,
	.dist = 0.34
};


static inline time_t quad_timeMsGet(void)
{
	time_t now;

	gettime(&now, NULL);

	return now / 1000;
}


static void quad_controlMotors(int32_t alt, int32_t roll, int32_t pitch, int32_t yaw)
{
	time_t dt, now;
	ekf_state_t measure;
	float palt, proll, ppitch, pyaw;

	ekf_stateGet(&measure);

	syslog(LOG_INFO, "EKF - x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n",
		measure.enuX, measure.enuY, measure.enuZ, measure.roll, measure.pitch, measure.yaw);

	now = quad_timeMsGet();
	dt = now - quad_common.lastTime;
	quad_common.lastTime = now;

#if TEST_ATTITUDE
	alt = measure.enuZ * 1000;
	yaw = measure.yaw;
#endif

	palt = pid_calc(&quad_common.pids.alt, measure.enuZ * 1000, alt, dt);
	proll = pid_calc(&quad_common.pids.roll, measure.roll, roll, dt);
	ppitch = pid_calc(&quad_common.pids.pitch, measure.pitch, pitch, dt);
	pyaw = pid_calc(&quad_common.pids.yaw, measure.yaw, yaw, dt);

	mma_control(palt, proll, ppitch, pyaw);

	syslog(LOG_INFO, "PIDs - alt: %f, roll: %f, pitch: %f, yaw: %f\n", palt, proll, ppitch, pyaw);
}


static int quad_takeoff(const flight_mode_t *mode)
{
	syslog(LOG_INFO, "TAKEOFF - alt: %d\n", mode->hover.alt);

	/* TBD */

	return 0;
}


static int quad_hover(const flight_mode_t *mode)
{
	time_t now;
	ekf_state_t state;

	syslog(LOG_INFO, "HOVER - alt: %d, time: %ld\n", mode->hover.alt, mode->hover.time);

	now = quad_timeMsGet();

#if TEST_ATTITUDE
	while (quad_timeMsGet() < now + mode->hover.time) {
		quad_controlMotors(mode->hover.alt, 0, 0, 0);
	}
#else
	ekf_stateGet(&state);
	while ((quad_timeMsGet() < now + mode->hover.time) || DELTA(state.enuZ, mode->hover.alt) > ALTITUDE_TOLERANCE) {
		quad_controlMotors(mode->hover.alt, 0, 0, 0);
		ekf_stateGet(&state);
	}
#endif

	return 0;
}


static int quad_landing(const flight_mode_t *mode)
{
	syslog(LOG_INFO, "LANDING\n");

	/* TBD */

	return 0;
}


static int quad_run(void)
{
	unsigned int i;

	quad_common.lastTime = quad_timeMsGet();

	for (i = 0; i < sizeof(scenario) / sizeof(scenario[0]); ++i) {
		switch (scenario[i].type) {
			case flight_takeoff:
				quad_takeoff(&scenario[i]);
				break;

			case flight_hover:
				quad_hover(&scenario[i]);
				break;

			case flight_landing:
				quad_landing(&scenario[i]);
				break;

			case flight_end:
				syslog(LOG_INFO, "end of the scenario\n");
				break;

			default:
				break;
		}
	}

	return 0;
}


static void quad_done(void)
{
	mma_stop();
	mma_done();

	ekf_done();
	closelog();
}


static int quad_init(void)
{
	/* PID initialization alt, roll, pitch, yaw */
	/* TODO: get data from config file: PATH_PIDS_CONFIG */

	/* MMA initialization */
	if (mma_init(&quadCoeffs) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize mma module\n");
		return -1;
	}

	/* EKF initialization */
	ekf_init();
	ekf_run();

	return 0;
}


int main(int argc, char **argv)
{
	int err;

	openlog("quad-control", LOG_NDELAY, LOG_DAEMON);

	err = quad_init();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	quad_run();
	quad_done();

	return EXIT_SUCCESS;
}
