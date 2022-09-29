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
#include "log.h"

#include <ekflib.h>

#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>


/* Flag enable hackish code for initial tests which ignore altitude and yaw */
#define TEST_ATTITUDE 1

#define DELTA(measurement, setVal) (fabs((setVal) - (measurement)))
#define ALTITUDE_TOLERANCE         500 /* 1E-3 [m] (millimetres) */

#define PATH_PIDS_CONFIG "/etc/quad.conf"
#define PID_NUMBERS      4

#define ANGLE_THRESHOLD (M_PI / 6)
#define RAD2DEG         ((float)180.0 / M_PI)
#define DEG2RAD         0.0174532925f
#define ANGLE_HOLD      INT32_MAX

#define LOG_PERIOD 50 /* drone control loop logs data once per 'LOG_PERIOD' milliseconds */


struct {
	pid_ctx_t pids[PID_NUMBERS];

	time_t lastTime;
#if TEST_ATTITUDE
	quad_att_t targetAtt;
	time_t duration;
#endif

	struct {
		float min;
		float max;
	} throttle;
} quad_common;


enum { pwm_alt = 0, pwm_roll, pwm_pitch, pwm_yaw, pwm_max };


/* Example of flight scenario.
   TODO: move data to the configuration file placed in a rootfs */
#if TEST_ATTITUDE
static const flight_mode_t scenario[] = {
	{ .type = flight_takeoff, .takeoff = { .alt = 5000, .time = 6000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_hover, .hover = { .alt = -2000, .time = 6000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_hover, .hover = { .alt = -2000, .time = 6000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_landing },
	{ .type = flight_end },
};
#else
static const flight_mode_t scenario[] = {
	{ .type = flight_takeoff, .takeoff = { .alt = 5000, .time = 2000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_hover, .hover = { .alt = 0, .time = 6000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_hover, .hover = { .alt = 0, .time = 6000 } },
	{ .type = flight_hover, .hover = { .alt = 4000, .time = 5000 } },
	{ .type = flight_landing },
	{ .type = flight_end },
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


static int quad_motorsCtrl(float throttle, int32_t alt, int32_t roll, int32_t pitch, int32_t yaw)
{
	time_t dt, now;
	ekf_state_t measure;
	float palt, proll, ppitch, pyaw;

	ekf_stateGet(&measure);

	if (fabs(measure.pitch) > ANGLE_THRESHOLD || fabs(measure.roll) > ANGLE_THRESHOLD) {
		fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors stop.\n", measure.roll, measure.pitch);
		mma_stop();
		return -1;
	}

	now = quad_timeMsGet();

	dt = now - quad_common.lastTime;
	quad_common.lastTime = now;

	if (yaw == ANGLE_HOLD) {
		yaw = measure.yaw * 1000;
	}


	log_print("EKFE: %lld %.1f %.1f %.1f\n", now, measure.yaw * RAD2DEG, measure.pitch * RAD2DEG, measure.roll * RAD2DEG);
	log_print("EKFX: %.2f\n", measure.enuZ);
	log_print("PID: ");


	palt = pid_calc(&quad_common.pids[pwm_alt], alt / 1000.f, measure.enuZ, 0, dt);
	proll = pid_calc(&quad_common.pids[pwm_roll], roll / 1000.f, measure.roll, measure.rollDot, dt);
	ppitch = pid_calc(&quad_common.pids[pwm_pitch], pitch / 1000.f, measure.pitch, measure.pitchDot, dt);
	pyaw = pid_calc(&quad_common.pids[pwm_yaw], yaw / 1000.f, measure.yaw, measure.yawDot, dt);
	log_print("\n");

	if (mma_control(throttle + palt, proll, ppitch, pyaw) < 0) {
		return -1;
	}

	usleep(1000 * 2);

	return 0;
}


static int quad_takeoff(const flight_mode_t *mode)
{
	float throttle, coeff;
	time_t spoolStart, spoolEnd, now, lastLog = 0;

	log_enable();
	log_print("TAKEOFF - alt: %d\n", mode->hover.alt);

	spoolStart = now = quad_timeMsGet();
	spoolEnd = spoolStart + mode->takeoff.time;

	while (now < spoolEnd) {
		now = quad_timeMsGet();

		/* Enable logging once per 'LOG_PERIOD' milliseconds */
		if (now - lastLog > LOG_PERIOD) {
			lastLog = now;
			log_enable();
		} else {
			log_disable();
		}

		coeff = (float)(now - spoolStart) / mode->takeoff.time;
		throttle = coeff * quad_common.throttle.max;

		if (quad_motorsCtrl(throttle, mode->takeoff.alt, 0, 0, ANGLE_HOLD) < 0) {
			return -1;
		}
	}

	/* TBD */

	return 0;
}


static int quad_hover(const flight_mode_t *mode)
{
	time_t now, end, lastLog;
	ekf_state_t state;

	log_enable();
	log_print("HOVER - alt: %d, time: %lld\n", mode->hover.alt, mode->hover.time);

	now = quad_timeMsGet();
	end = now + mode->hover.time;
	lastLog = 0;

#if TEST_ATTITUDE
	while (now < end) {
		/* Enable logging once per 'LOG_PERIOD' milliseconds */
		if (now - lastLog > LOG_PERIOD) {
			lastLog = now;
			log_enable();
		}
		else {
			log_disable();
		}

		if (quad_motorsCtrl(quad_common.throttle.max, mode->hover.alt, quad_common.targetAtt.roll, quad_common.targetAtt.pitch, quad_common.targetAtt.yaw) < 0) {
			return -1;
		}

		now = quad_timeMsGet();
	}
#else
	ekf_stateGet(&state);
	while ((quad_timeMsGet() < now + mode->hover.time) || DELTA(state.enuZ, mode->hover.alt) > ALTITUDE_TOLERANCE) {
		if (quad_motorsCtrl(quad_common.throttle.max, mode->hover.alt, 0, 0, 0) < 0) {
			return -1;
		}
		ekf_stateGet(&state);
	}
#endif

	return 0;
}


static int quad_landing(const flight_mode_t *mode)
{
	float coeff;
	log_print("LANDING\n");

	/* Soft landing */
	for (coeff = 1.0; coeff > 0.00001; coeff -= 0.02f) {
#if TEST_ATTITUDE
		if (quad_motorsCtrl(coeff * quad_common.throttle.max, 0, 0, 0, ANGLE_HOLD) < 0) {
			return -1;
		}
#endif
		/* TODO: do not change throttle value, decrease gradually altitude to change throttle */
		usleep(1000 * 100);
	}

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
				/* Arm motors */
				mma_start();
				err = quad_takeoff(&scenario[i]);
				break;

			case flight_hover:
				err = quad_hover(&scenario[i]);
				break;

			case flight_landing:
				err = quad_landing(&scenario[i]);
				break;

			case flight_end:
				log_print("end of the scenario\n");
				/* Disarm motors */
				mma_stop();
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


/* Initialization functions */

static inline int quad_divLine(char **line, char **var, float *val)
{
	char *strVal;

	*var = *line;
	strVal = strchr(*line, '=');
	if (strVal == NULL) {
		return -EINVAL;
	}

	*strVal++ = '\0';
	*val = strtof(strVal, NULL);

	return EOK;
}

#if TEST_ATTITUDE
static int quad_attParse(FILE *file)
{
	int err = EOK;
	float val;
	size_t len = 0;
	unsigned int cnt = 0;
	const unsigned int fieldsNb = 3;
	char *line = NULL, *var;

	while (getline(&line, &len, file) != -1 && cnt < fieldsNb) {
		err = quad_divLine(&line, &var, &val);
		if (err < 0) {
			fprintf(stderr, "quad-control: attitude wrong line %s in file %s\n", line, PATH_PIDS_CONFIG);
			break;
		}

		if (strcmp(var, "YAW") == 0) {
			quad_common.targetAtt.yaw = val * DEG2RAD * 1000;
		}
		else if (strcmp(var, "PITCH") == 0) {
			quad_common.targetAtt.pitch = val * DEG2RAD * 1000;
		}
		else if (strcmp(var, "ROLL") == 0) {
			quad_common.targetAtt.roll = val * DEG2RAD * 1000;
		}
		else {
			fprintf(stderr, "quad-control: attitude wrong variable %s\n", var);
			err = -EINVAL;
			break;
		}

		++cnt;
	}

	if (cnt != fieldsNb) {
		err = -EINVAL;
	}

	free(line);

	return err;
}
#endif

static int quad_pidParse(FILE *file, unsigned int i)
{
	int err = EOK;
	float val;
	size_t len = 0;
	pid_ctx_t *pid;
	unsigned int cnt = 0;
	const unsigned int fieldsNb = 7;
	char *line = NULL, *var;

	if (i >= PID_NUMBERS) {
		return -EINVAL;
	}

	while (getline(&line, &len, file) != -1 && cnt < fieldsNb) {
		err = quad_divLine(&line, &var, &val);
		if (err < 0) {
			fprintf(stderr, "quad-control: pid wrong line %s in file %s\n", line, PATH_PIDS_CONFIG);
			break;
		}

		pid = &quad_common.pids[i];

		if (strcmp(var, "P") == 0) {
			pid->kp = val;
		}
		else if (strcmp(var, "I") == 0) {
			pid->ki = val;
		}
		else if (strcmp(var, "D") == 0) {
			pid->kd = val;
		}
		else if (strcmp(var, "MIN") == 0) {
			pid->min = val;
		}
		else if (strcmp(var, "MAX") == 0) {
			pid->max = val;
		}
		else if (strcmp(var, "IMAX") == 0) {
			pid->maxInteg = val;
		}
		else if (strcmp(var, "IMIN") == 0) {
			pid->minInteg = val;
		}
		else {
			fprintf(stderr, "quad-control: pid wrong variable %s\n", var);
			err = -EINVAL;
			break;
		}

		++cnt;
	}

	if (cnt != fieldsNb) {
		err = -EINVAL;
	}

	free(line);

	return err;
}


static int quad_throttleParse(FILE *file)
{
	int err = EOK;
	float val;
	size_t len = 0;
	unsigned int cnt = 0;
	const unsigned int fieldsNb = 2;
	char *line = NULL, *var;

	while (getline(&line, &len, file) != -1 && cnt < fieldsNb) {
		err = quad_divLine(&line, &var, &val);
		if (err < 0) {
			fprintf(stderr, "quad-control: throttle wrong line %s in file %s\n", line, PATH_PIDS_CONFIG);
			break;
		}

		if (strcmp(var, "MIN") == 0) {
			quad_common.throttle.min = val;
		}
		else if (strcmp(var, "MAX") == 0) {
			quad_common.throttle.max = val;
		}
		else {
			fprintf(stderr, "quad-control: throttle wrong variable %s\n", var);
			err = -EINVAL;
			break;
		}

		++cnt;
	}

	if (cnt != fieldsNb) {
		err = -EINVAL;
	}

	free(line);

	return err;
}


static int quad_configRead(void)
{
	FILE *file;
	int err = EOK;
	size_t len = 0;
	char *line = NULL;
	unsigned int pidCnt = 0;

	file = fopen(PATH_PIDS_CONFIG, "r");
	if (file == NULL) {
		return -EBADFD;
	}

	while (getline(&line, &len, file) != -1) {
		if ((line[0] == '\n') || (line[0] == '#')) {
			continue;
		}

		/* @PID - define new PID */
		if ((line[0] == '@') && (strcmp(&line[1], "PID\n") == 0)) {
			err = quad_pidParse(file, pidCnt++);
			if (err < 0) {
				fprintf(stderr, "quad-control: cannot parse pid parameters\n");
				break;
			}
		}

		/* @THROTTLE - define throttle min & max values */
		if ((line[0] == '@') && (strcmp(&line[1], "THROTTLE\n") == 0)) {
			err = quad_throttleParse(file);
			if (err < 0) {
				fprintf(stderr, "quad-control: cannot parse throttle parameters\n");
				break;
			}
		}
#if TEST_ATTITUDE
		/* @ATTITUDE - define target attitude */
		if ((line[0] == '@') && (strcmp(&line[1], "ATTITUDE\n") == 0)) {
			err = quad_attParse(file);
			if (err < 0) {
				fprintf(stderr, "quad-control: cannot parse attitude parameters\n");
				break;
			}
		}
#endif
	}

	free(line);
	fclose(file);

	return err;
}


static void quad_done(void)
{
	mma_done();
	ekf_done();
}


static int quad_init(void)
{
	unsigned int i = 0;

	/* Enabling logging by default */
	log_enable();

	/* PID initialization alt, roll, pitch, yaw */
	if (quad_configRead() < 0) {
		fprintf(stderr, "quadcontrol: cannot parse %s\n", PATH_PIDS_CONFIG);
		return -1;
	}

	/* Set initial values */
	for (i = 0; i < PID_NUMBERS; ++i) {
		if (pid_init(&quad_common.pids[i]) < 0) {
			fprintf(stderr, "quadcontrol: cannot initialize PID %d\n", i);
			return -1;
		}
	}
	/* get boundary values of euler angles from ekf module */
	ekf_boundsGet(&quad_common.pids[pwm_yaw].errBound, &quad_common.pids[pwm_roll].errBound, &quad_common.pids[pwm_pitch].errBound);

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
	pid_t pid, ret;
	int status = 0;

	priority(1);

	err = quad_init();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	/* Flight duration is get only for tests */
#if TEST_ATTITUDE
	if (argc != 2) {
		fprintf(stderr, "quadcontrol: app is in TEST MODE, provide test duration in ms\n");
		return EXIT_FAILURE;
	}
	quad_common.duration = strtoul(argv[1], NULL, 0);
#endif

	signal(SIGINT, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	signal(SIGTERM, SIG_IGN);

	pid = vfork();
	/* Parent process waits on child and makes clean up  */
	if (pid > 0) {
		do {
			ret = waitpid(pid, &status, 0);
		} while (ret < 0 && errno == EINTR);
	}
	/* Child process runs flight scenario and can be terminated */
	else if (pid == 0) {
		signal(SIGINT, SIG_DFL);
		signal(SIGQUIT, SIG_DFL);
		signal(SIGTERM, SIG_DFL);

		/* Take terminal control */
		tcsetpgrp(STDIN_FILENO, getpid());

		quad_run();
		exit(EXIT_SUCCESS);
	}
	else {
		fprintf(stderr, "quadcontrol: vfork failed with code %d\n", pid);
	}

	quad_done();

	/* Take back terminal control */
	tcsetpgrp(STDIN_FILENO, getpid());

	return EXIT_SUCCESS;
}
