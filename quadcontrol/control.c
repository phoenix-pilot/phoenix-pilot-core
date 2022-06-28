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

#include <math.h>
#include <ekflib.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/threads.h>


#define DELTA(measurement, setVal) (fabs((setVal) - (measurement)))
#define ALTITUDE_TOLERANCE         500 /* 1E-3 [m] (millimetres) */

#define PATH_PIDS_CONFIG "/etc/quad.conf"
#define PID_NUMBERS      4

/* Flag enable hackish code for initial tests which ignore altitude and yaw */
#define TEST_ATTITUDE   1
#define ANGLE_THRESHOLD (M_PI / 6)
#define RAD2DEG         ((float)180.0 / M_PI)


struct {
	pid_ctx_t pids[PID_NUMBERS];

	time_t lastTime;
	time_t duration;
} quad_common;


enum { pwm_alt = 0, pwm_roll, pwm_pitch, pwm_yaw, pwm_max };


/* Example of flight scenario.
   TODO: move data to the configuration file placed in a rootfs */
#if TEST_ATTITUDE
static const flight_mode_t scenario[] = {
	{ .type = flight_takeoff, .takeoff = { .alt = 2000 } },
	{ .type = flight_hover, .hover = { .alt = 2000, .time = 10000 } },
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


static void quad_done(void)
{
	mma_stop();
	mma_done();

	ekf_done();
	closelog();
}


static inline time_t quad_timeMsGet(void)
{
	time_t now;

	gettime(&now, NULL);

	return now / 1000;
}


static int quad_controlMotors(int32_t alt, int32_t roll, int32_t pitch, int32_t yaw)
{
	time_t dt, now;
	ekf_state_t measure;
	float palt, proll, ppitch, pyaw;

	ekf_stateGet(&measure);

	if (fabs(measure.pitch) > ANGLE_THRESHOLD || fabs(measure.roll) > ANGLE_THRESHOLD) {
		fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors OFF\n", measure.roll, measure.pitch);
		mma_control(0, 0, 0, 0);
		return -1;
	}

	now = quad_timeMsGet();
	DEBUG_LOG("EKF: %lld, %f, %f, %f, %f\n", now, measure.yaw * RAD2DEG, measure.roll * RAD2DEG, measure.pitch * RAD2DEG, measure.enuZ);

	dt = now - quad_common.lastTime;
	quad_common.lastTime = now;

#if TEST_ATTITUDE
	alt = measure.enuZ * 1000;
	yaw = measure.yaw;
	palt = quad_common.pids[pwm_alt].kp;
#else
	palt = pid_calc(&quad_common.pids[pwm_alt], measure.enuZ * 1000, alt, dt);
#endif

	DEBUG_LOG("PID: %lld, ", now);
	proll = pid_calc(&quad_common.pids[pwm_roll], roll, measure.roll, dt);
	ppitch = pid_calc(&quad_common.pids[pwm_pitch], pitch, measure.pitch, dt);
	pyaw = pid_calc(&quad_common.pids[pwm_yaw], yaw, measure.yaw, dt);
	DEBUG_LOG("\n");

	mma_control(palt, proll, ppitch, pyaw);

	usleep(1000 * 2);

	return 0;
}


static int quad_takeoff(const flight_mode_t *mode)
{
	time_t dt, now;
	float thrust, coeff;
	ekf_state_t measure;
	float palt, proll, ppitch, pyaw;

	DEBUG_LOG("TAKEOFF - alt: %d\n", mode->hover.alt);

	/* Soft motors start */
	for (coeff = 0, thrust = 0; thrust < quad_common.pids[pwm_alt].kp; coeff += 0.01f) {
		now = quad_timeMsGet();
		ekf_stateGet(&measure);

		if (fabs(measure.pitch) > ANGLE_THRESHOLD || fabs(measure.roll) > ANGLE_THRESHOLD) {
			fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors OFF\n", measure.roll, measure.pitch);
			mma_control(0, 0, 0, 0);
			return -1;
		}

		dt = now - quad_common.lastTime;
		quad_common.lastTime = now;

		DEBUG_LOG("EKF: %lld, %f, %f, %f, %f\n", now, measure.yaw * RAD2DEG, measure.roll * RAD2DEG, measure.pitch * RAD2DEG, measure.enuZ);

		DEBUG_LOG("PID: %lld, ", now);
		proll = pid_calc(&quad_common.pids[pwm_roll], 0, measure.roll, dt);
		ppitch = pid_calc(&quad_common.pids[pwm_pitch], 0, measure.pitch, dt);
		pyaw = pid_calc(&quad_common.pids[pwm_yaw], 0, measure.yaw, dt);
		DEBUG_LOG("\n");

		thrust = coeff * quad_common.pids[pwm_alt].kp;
		mma_control(thrust, coeff * proll, coeff * ppitch, coeff * pyaw);

		usleep(1000 * 200);
	}

	/* TBD */

	return 0;
}


static int quad_hover(const flight_mode_t *mode)
{
	time_t now;
	ekf_state_t state;

	DEBUG_LOG("HOVER - alt: %d, time: %lld\n", mode->hover.alt, mode->hover.time);

	now = quad_timeMsGet();

#if TEST_ATTITUDE
	while (quad_timeMsGet() < now + quad_common.duration) {
		if (quad_controlMotors(mode->hover.alt, 0, 0, 0) < 0) {
			return -1;
		}
	}
#else
	ekf_stateGet(&state);
	while ((quad_timeMsGet() < now + mode->hover.time) || DELTA(state.enuZ, mode->hover.alt) > ALTITUDE_TOLERANCE) {
		if (quad_controlMotors(mode->hover.alt, 0, 0, 0) < 0) {
			return -1;
		}
		ekf_stateGet(&state);
	}
#endif

	return 0;
}


static int quad_landing(const flight_mode_t *mode)
{
	DEBUG_LOG("LANDING\n");

	/* TBD */

	return 0;
}


static int quad_run(void)
{
	int err = 0;
	unsigned int i;

	quad_common.lastTime = quad_timeMsGet();

	for (i = 0; i < sizeof(scenario) / sizeof(scenario[0]); ++i) {
		switch (scenario[i].type) {
			case flight_takeoff:
				err = quad_takeoff(&scenario[i]);
				break;

			case flight_hover:
				err = quad_hover(&scenario[i]);
				break;

			case flight_landing:
				err = quad_landing(&scenario[i]);
				break;

			case flight_end:
				DEBUG_LOG("end of the scenario\n");
				mma_control(0, 0, 0, 0);
				break;

			default:
				break;
		}

		if (err < 0) {
			break;
		}
	}

	return err;
}


static int quad_parsePid(unsigned int i, const char *conf, float val)
{
	int err = EOK;
	pid_ctx_t *pid;

	if (i >= PID_NUMBERS) {
		return -EINVAL;
	}

	pid = &quad_common.pids[i];

	if (strcmp(conf, "P") == 0) {
		pid->kp = val;
	}
	else if (strcmp(conf, "I") == 0) {
		pid->ki = val;
	}
	else if (strcmp(conf, "D") == 0) {
		pid->kd = val;
	}
	else if (strcmp(conf, "MIN") == 0) {
		pid->min = val;
	}
	else if (strcmp(conf, "MAX") == 0) {
		pid->max = val;
	}
	else if (strcmp(conf, "IMAX") == 0) {
		pid->maxInteg = val;
	}
	else if (strcmp(conf, "IMIN") == 0) {
		pid->minInteg = val;
	}
	else {
		err = -EINVAL;
	}

	return err;
}


static int quad_readConfig(void)
{
	float val;
	FILE *file;
	int err = EOK;
	unsigned int i = 0;
	size_t len = 0;
	char *line = NULL, *var, *strVal;

	file = fopen(PATH_PIDS_CONFIG, "r");
	if (file == NULL) {
		return -EBADFD;
	}

	while (getline(&line, &len, file) != -1) {
		/* @ - define new PID */
		if ((line[0] == '@') && (strcmp(&line[1], "PID\n") == 0)) {
			i++;
			continue;
		}

		if ((line[0] == '\n') || (line[0] == '#')) {
			continue;
		}

		var = line;
		strVal = strchr(line, '=');
		if (strVal == NULL) {
			fprintf(stderr, "quad-control: wrong line %s in file %s\n", line, PATH_PIDS_CONFIG);
			err = -EINVAL;
			break;
		}

		*strVal++ = '\0';
		val = strtof(strVal, NULL);

		if (quad_parsePid(i - 1, var, val) < 0) {
			fprintf(stderr, "quad-control: cannot parse %s in file %s\n", var, PATH_PIDS_CONFIG);
			err = -EINVAL;
			break;
		}
	}

	free(line);
	fclose(file);

	return err;
}


static int quad_init(void)
{
	unsigned int i = 0;

	/* PID initialization alt, roll, pitch, yaw */
	if (quad_readConfig() < 0) {
		fprintf(stderr, "quadcontrol: cannot parse %s\n", PATH_PIDS_CONFIG);
		return -1;
	}

	/* Set initial values */
	for (i = 0; i < PID_NUMBERS; ++i) {
		pid_init(&quad_common.pids[i]);
	}

	/* MMA initialization */
	if (mma_init(&quadCoeffs) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize mma module\n");
		return -1;
	}

	/* EKF initialization */
	if (ekf_init() < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize ekf\n");
		return -1;
	}

	if (ekf_run() < 0) {
		fprintf(stderr, "quadcontrol: cannot run ekf\n");
		return -1;
	}

	/* EKF needs time to calibrate itself */
	sleep(10);

	return 0;
}


int main(int argc, char **argv)
{
	int err;
	quad_common.duration = 10000;

	priority(1);

	/* Flight duration is get only for tests */
	if (argc != 2) {
		return 1;
	}
	quad_common.duration = strtoul(argv[1], NULL, 0);

	err = quad_init();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	quad_run();
	quad_done();

	return EXIT_SUCCESS;
}
