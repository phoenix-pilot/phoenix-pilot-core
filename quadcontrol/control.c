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
#include "config.h"

#include <ekflib.h>
#include <rcbus.h>

#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include <board_config.h>


/* Flag enable hackish code for initial tests which ignore altitude and yaw */
#define TEST_ATTITUDE 1

#define DELTA(measurement, setVal) (fabs((setVal) - (measurement)))
#define ALTITUDE_TOLERANCE         500 /* 1E-3 [m] (millimetres) */

#define PATH_QUAD_CONFIG "/etc/quad.conf"
#define PID_NUMBERS      4

#define PATH_SCENARIO_CONFIG "/etc/q_mission.conf"

#define HOVER_THROTTLE 0.27

#define ANGLE_THRESHOLD_LOW  (M_PI / 4) /* low threshold for landing safety */
#define ANGLE_THRESHOLD_HIGH (M_PI / 2) /* high threshold for maneuvers */
#define RAD2DEG              ((float)180.0 / M_PI)
#define DEG2RAD              0.0174532925f

/* rcbus trigger thresholds for manual switches SWA/SWB/SWC/SWD */
#define RCTHRESH_HIGH ((95 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) / 100 + MIN_CHANNEL_VALUE) /* high position threshold */
#define RCTHRESH_LOW  ((5 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) / 100 + MIN_CHANNEL_VALUE)  /* low position threshold */

#define ABORT_FRAMES_THRESH 5 /* number of correct abort frames from RC transmitter to initiate abort sequence */
#define LOG_PERIOD 100        /* drone control loop logs data once per 'LOG_PERIOD' milliseconds */


typedef enum { mode_rc = 0, mode_auto } control_mode_t;

struct {
	pid_ctx_t *pids;

	handle_t rcbusLock;
	uint16_t rcChannels[RC_CHANNELS_CNT];

	volatile control_mode_t mode;
	volatile flight_type_t currFlight;

	time_t lastTime;
#if TEST_ATTITUDE
	quad_att_t targetAtt;
#endif

	flight_mode_t *scenario;
	size_t scenarioSz;

	quad_throttle_t throttle;
} quad_common;


enum { pwm_alt = 0, pwm_roll, pwm_pitch, pwm_yaw, pwm_max };


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


static inline void quad_pidRestore(void)
{
	quad_common.pids[pwm_alt].flags = PID_FULL;
	quad_common.pids[pwm_roll].flags = PID_FULL;
	quad_common.pids[pwm_pitch].flags = PID_FULL;
	quad_common.pids[pwm_yaw].flags = PID_FULL;
}


/* Sets logging on/off and returns current logging state */
static inline bool quad_periodLogEnable(time_t now)
{
	static time_t lastLog = 0;

	/* Enable logging once per 'LOG_PERIOD' milliseconds */
	if (now - lastLog > LOG_PERIOD) {
		lastLog = now;
		log_enable();
		return true;
	}

	log_disable();

	return false;
}


static int quad_motorsCtrl(float throttle, int32_t alt, const quad_att_t *att, const ekf_state_t *measure)
{
	time_t dt, now;
	float palt, proll, ppitch, pyaw;

	if (fabs(measure->pitch) > ANGLE_THRESHOLD_HIGH || fabs(measure->roll) > ANGLE_THRESHOLD_HIGH) {
		fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors stop.\n", measure->roll, measure->pitch);
		mma_stop();
		return -1;
	}

	now = quad_timeMsGet();

	dt = now - quad_common.lastTime;
	quad_common.lastTime = now;


	log_print("EKFE: %lld %.1f %.1f %.1f\n", now, measure->yaw * RAD2DEG, measure->pitch * RAD2DEG, measure->roll * RAD2DEG);
	log_print("EKFX: %.2f %.2f\n", measure->enuZ, measure->veloZ);
	log_print("PID: ");


	palt = pid_calc(&quad_common.pids[pwm_alt], alt / 1000.f, measure->enuZ, measure->veloZ, dt);
	proll = pid_calc(&quad_common.pids[pwm_roll], att->roll, measure->roll, measure->rollDot, dt);
	ppitch = pid_calc(&quad_common.pids[pwm_pitch], att->pitch, measure->pitch, measure->pitchDot, dt);
	pyaw = pid_calc(&quad_common.pids[pwm_yaw], att->yaw, measure->yaw, measure->yawDot, dt);
	log_print("\n");

	if (mma_control(throttle + palt, proll, ppitch, pyaw) < 0) {
		return -1;
	}

	usleep(1000);

	return 0;
}


/* Handling flight modes */

static int quad_takeoff(const flight_mode_t *mode)
{
	quad_att_t att = { 0 };
	float throttle, coeff;
	time_t spoolStart, spoolEnd, now;
	ekf_state_t measure;

	ekf_stateGet(&measure);

	att.yaw = measure.yaw;

	log_enable();
	log_print("TAKEOFF - alt: %d\n", mode->hover.alt);

	spoolStart = now = quad_timeMsGet();
	spoolEnd = spoolStart + mode->takeoff.time;

	while (now < spoolEnd && quad_common.currFlight < flight_manual) {
		ekf_stateGet(&measure);

		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		coeff = (float)(now - spoolStart) / mode->takeoff.time;
		throttle = coeff * quad_common.throttle.max;

		if (quad_motorsCtrl(throttle, mode->takeoff.alt, &att, &measure) < 0) {
			return -1;
		}
	}

	/* TBD */

	return 0;
}


static int quad_hover(const flight_mode_t *mode)
{
	quad_att_t att = { 0 };
	time_t now, end;
	ekf_state_t measure;

	ekf_stateGet(&measure);

	att.yaw = measure.yaw;

	log_enable();
	log_print("HOVER - alt: %d, time: %lld\n", mode->hover.alt, mode->hover.time);

	now = quad_timeMsGet();
	end = now + mode->hover.time;

	while (now < end && quad_common.currFlight < flight_manual) {
		ekf_stateGet(&measure);

		quad_periodLogEnable(now);

		if (quad_motorsCtrl(quad_common.throttle.max, mode->hover.alt, &att, &measure) < 0) {
			return -1;
		}

		now = quad_timeMsGet();
	}

	return 0;
}


static int quad_landing(const flight_mode_t *mode)
{
	quad_att_t att = { 0 };
	ekf_state_t measure;
	float throttle, coeff;
	time_t spoolStart, spoolEnd, now;

	ekf_stateGet(&measure);

	att.yaw = measure.yaw;

	log_enable();
	log_print("LANDING\n");

	spoolStart = now = quad_timeMsGet();
	spoolEnd = spoolStart + mode->landing.time;

	while (now < spoolEnd && quad_common.currFlight < flight_manual) {
		ekf_stateGet(&measure);

		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		coeff = (float)(now - spoolStart) / mode->landing.time;
		throttle = (1.f - coeff) * quad_common.throttle.max;

		if (quad_motorsCtrl(throttle, 0, &att, &measure) < 0) {
			return -1;
		}
	}

	return 0;
}


/*
* quad_manual allows for manual and semi manual control over drone in STABILIZE and ALTHOLD modes:
*
* STABILIZE mode provide:
* - control over throttle in range (50%, 150%) of HOVER_THROTTLE
* - control over roll/pitch
* - gathering data of measured altitude
*
* ALTHOLD mode provide:
* - keeps last STABILIZE altitude
* - control over roll/pitch
* - throttle is left as was in STABILIZE mode
*/
static int quad_manual(void)
{
	quad_att_t att;
	ekf_state_t measure;
	float throttle, hoverThrtlEst = quad_common.throttle.max;
	time_t now;
	int32_t alt, yawDelta;
	int32_t rcRoll, rcPitch, rcThrottle, rcYaw, stickSWD;
	bool althold;

	ekf_stateGet(&measure);
	alt = measure.enuZ * 1000;
	att.yaw = measure.yaw;

	log_enable();
	log_print("RC Control\n");

	/* Read RC packet and do initialization check */
	mutexLock(quad_common.rcbusLock);
	stickSWD = quad_common.rcChannels[RC_SWD_CH];
	rcThrottle = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
	mutexUnlock(quad_common.rcbusLock);

	if (stickSWD >= RCTHRESH_LOW || rcThrottle >= RCTHRESH_LOW) {
		fprintf(stderr, "Wrong initial transmitter setup. Aborting...\n");
		return -1;
	}

	throttle = hoverThrtlEst * ((float)rcThrottle / (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) + 0.5); /* throttle now in range (0.5, 1.5) of hover throttle */

	now = quad_timeMsGet();
	while (quad_common.currFlight == flight_manual) {
		ekf_stateGet(&measure);

		now = quad_timeMsGet();

		quad_periodLogEnable(now);

		mutexLock(quad_common.rcbusLock);
		stickSWD = quad_common.rcChannels[RC_SWD_CH];
		rcRoll = quad_common.rcChannels[RC_RIGHT_HSTICK_CH];
		rcPitch = quad_common.rcChannels[RC_RIGHT_VSTICK_CH];
		rcYaw = quad_common.rcChannels[RC_LEFT_HSTICK_CH];
		rcThrottle = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
		mutexUnlock(quad_common.rcbusLock);

		althold = (stickSWD > 0.9 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) ? true : false;
		yawDelta = rcYaw - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2); /* delta yaw in +-500 milliradians range */

		att.roll = (rcRoll - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2)) / 1000.f; /* +-0.5 radian ~= +- 28 degrees */
		/* minus in pitch is to compensate for rc transmitter sign: stick up => higher channel value => negative pitch change */
		att.pitch = -(rcPitch - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2)) / 1000.f; /* +-0.5 radian ~= +- 28 degrees */

		/* Update target yaw/altitude only if we are not in althold */
		if (althold == false) {
			throttle = hoverThrtlEst * ((float)rcThrottle / (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) + 0.5); /* throttle now in range (0.5, 1.5) of hover throttle */

			/* setting current value as target value, so that PID controller uses only D term */
			alt = measure.enuZ * 1000;
			att.yaw = measure.yaw + (yawDelta / 1000.f);
		}

		/* Perform low threshold check only if throttle is at minimum (probable landing) in case of drone tipping off */
		if (rcThrottle < 0.05 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) {
			if (fabs(measure.pitch) > ANGLE_THRESHOLD_LOW || fabs(measure.roll) > ANGLE_THRESHOLD_LOW) {
				fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors stop.\n", measure.roll, measure.pitch);
				mma_stop();
				return -1;
			}
		}

		/* Disabling altitude pid controller in STABILIZE for full RC throttle control */
		quad_common.pids[pwm_alt].flags = PID_FULL;
		if (althold == false) {
			quad_common.pids[pwm_alt].flags |= PID_IGNORE_P | PID_IGNORE_I | PID_IGNORE_D | PID_RESET_I;
		}

		if (quad_motorsCtrl(throttle, alt, &att, &measure) < 0) {
			return -1;
		}
	}

	return 0;
}


static int quad_run(void)
{
	int err = 0, armed = 0;
	unsigned int i = 0, run = 1;

	quad_common.lastTime = quad_timeMsGet();

	while (i < quad_common.scenarioSz && run == 1) {

		if ((quad_common.currFlight >= flight_arm && armed == 1) && quad_common.currFlight < flight_manual) {
			quad_common.currFlight = quad_common.scenario[i].type;
		}

		quad_pidRestore();

		log_enable();
		switch (quad_common.currFlight) {
			/* Handling basic modes */
			case flight_idle:
				if (armed != 0) {
					log_print("f_disarm: disarming motors...\n");
					mma_stop();
					armed = 0;
				}
				log_print("f_idle: idling...\n");
				sleep(1);
				break;

			case flight_disarm:
				if (armed != 0) {
					log_print("f_disarm: disarming motors...\n");
					mma_stop();
					armed = 0;
				}
				log_print("f_disarm: idling...\n");
				sleep(1);
				break;

			case flight_arm:
				if (armed == 0) {
					log_print("f_arm: arming motors...\n");
					mma_start();
					armed = 1;
				}
				/* TODO: enable buzzer */
				sleep(1);

				if (quad_common.mode == mode_rc) {
					quad_common.currFlight = flight_manual;
				}
				break;

			/* Handling auto modes: */
			case flight_takeoff:
				log_print("f_takeoff\n");
				err = quad_takeoff(&quad_common.scenario[i++]);
				break;

			case flight_pos:
				log_print("f_pos\n");
				/* TBD */
				break;

			case flight_hover:
				log_print("f_hover\n");
				err = quad_hover(&quad_common.scenario[i++]);
				break;

			case flight_landing:
				log_print("f_landing\n");
				err = quad_landing(&quad_common.scenario[i++]);
				break;

			case flight_end:
				log_print("f_end\n");
				mma_stop();
				armed = 0;
				run = 0;
				break;

			/* Handling manual modes: */
			case flight_manual:
				log_print("f_manual\n");
				quad_common.mode = mode_rc;
				err = quad_manual();
				break;

			case flight_manualAbort:
				log_print("Mission abort\n");
				mma_stop();
				armed = 0;
				run = 0;
				break;

			default:
				log_print("f_unknown\n");
				break;
		}

		if (err < 0) {
			/* Disarm motors */
			mma_stop();
			break;
		}
	}

	return err;
}


static void quad_rcbusHandler(const rcbus_msg_t *msg)
{
	/* abort frame counting variable*/
	static unsigned int abortCnt = 0;

	if (msg->channelsCnt < RC_CHANNELS_CNT) {
		fprintf(stderr, "quad-control: rcbus supports insufficient number of channels\n");
		return;
	}

	/* Manual Disarm: SWA == MIN, SWB == MIN, SWC == MIN, SWD == MIN, Throttle == 0 && mode_rc */
	if (msg->channels[RC_SWA_CH] <= RCTHRESH_LOW && msg->channels[RC_SWB_CH] <= RCTHRESH_LOW
			&& msg->channels[RC_SWC_CH] <= RCTHRESH_LOW && msg->channels[RC_SWD_CH] <= RCTHRESH_LOW
			&& msg->channels[RC_LEFT_VSTICK_CH] <= RCTHRESH_LOW && quad_common.currFlight == flight_idle) {
		printf("rc: set f_disarm\n");
		quad_common.currFlight = flight_disarm;
	}
	/* Manual Arm: SWA == MAX, SWB == MIN and Throttle == 0 and scenario cannot be launched */
	else if (msg->channels[RC_SWA_CH] >= RCTHRESH_HIGH && msg->channels[RC_LEFT_VSTICK_CH] <= RCTHRESH_LOW
			&& quad_common.currFlight == flight_disarm) {
		printf("rc: set f_arm\n");
		quad_common.currFlight = flight_arm;
	}
	/* Emergency abort: SWA == MAX, SWB == MAX, SWC == MAX, SWD == MAX */
	else if (msg->channels[RC_SWA_CH] >= RCTHRESH_HIGH && msg->channels[RC_SWB_CH] >= RCTHRESH_HIGH
			&& msg->channels[RC_SWC_CH] >= RCTHRESH_HIGH && msg->channels[RC_SWD_CH] >= RCTHRESH_HIGH 
			&& quad_common.currFlight < flight_manualAbort) {
		abortCnt++;
		printf("rc: f_abort called %i\n", abortCnt);

		if (abortCnt >= ABORT_FRAMES_THRESH) {
			printf("rc: f_abort reached\n");
			quad_common.currFlight = flight_manualAbort;
		}

		return;
	}
	/* Manual Mode: SWA == MAX, SWB == MAX */
	else if (msg->channels[RC_SWA_CH] >= RCTHRESH_HIGH && msg->channels[RC_SWB_CH] >= RCTHRESH_HIGH && quad_common.currFlight < flight_manual) {
		printf("rc: set f_manual\n");
		quad_common.currFlight = flight_manual;
	}

	/* Reset abort counter if new frame does not call for abort */
	abortCnt = 0;

	if (quad_common.mode == mode_rc) {
		mutexLock(quad_common.rcbusLock);
		memcpy(quad_common.rcChannels, msg->channels, sizeof(quad_common.rcChannels));
		mutexUnlock(quad_common.rcbusLock);
	}
}


static void quad_done(void)
{
	mma_stop();
	mma_done();
	ekf_done();
	rcbus_done();

	free(quad_common.scenario);
	free(quad_common.pids);

	resourceDestroy(quad_common.rcbusLock);
}


static int quad_config(void)
{
	int res;
	quad_throttle_t *throttleTmp;
#if TEST_ATTITUDE
	quad_att_t *attTmp;
#endif

	if (config_pidRead(PATH_QUAD_CONFIG, &quad_common.pids, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse PIDs from %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	if (res != PID_NUMBERS) {
		fprintf(stderr, "quadcontrol: wrong number of PIDs in %s", PATH_QUAD_CONFIG);
		free(quad_common.pids);
		return -1;
	}

	if (config_throttleRead(PATH_QUAD_CONFIG, &throttleTmp, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse throttle from %s\n", PATH_QUAD_CONFIG);
		free(quad_common.pids);
		return -1;
	}

	if (res != 1) {
		fprintf(stderr, "quadcontrol: wrong number of throttle configs in %s", PATH_QUAD_CONFIG);
		free(quad_common.pids);
		free(throttleTmp);
		return -1;
	}

	quad_common.throttle = *throttleTmp;
	free(throttleTmp);

#if TEST_ATTITUDE
	if (config_attitudeRead(PATH_QUAD_CONFIG, &attTmp, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse attitude from %s\n", PATH_QUAD_CONFIG);
		free(quad_common.pids);
		return -1;
	}

	if (res != 1) {
		fprintf(stderr, "quadcontrol: wrong number of attitude configs in %s", PATH_QUAD_CONFIG);
		free(quad_common.pids);
		free(attTmp);
		return -1;
	}

	quad_common.targetAtt = *attTmp;
	free(attTmp);
#endif

	if (config_scenarioRead(PATH_SCENARIO_CONFIG, &quad_common.scenario, &quad_common.scenarioSz) != 0) {
		free(quad_common.pids);
		return -1;
	}

	return 0;
}


static int quad_init(void)
{
	int res;
	unsigned int i = 0;

	/* Enabling logging by default */
	log_enable();

	/* Set idle flight mode */
	quad_common.currFlight = flight_idle;

	res = mutexCreate(&quad_common.rcbusLock);
	if (res < 0) {
		return res;
	}

	if (quad_config() != 0) {
		resourceDestroy(quad_common.rcbusLock);
		return -1;
	}

	/* Set initial values */
	for (i = 0; i < PID_NUMBERS; ++i) {
		if (pid_init(&quad_common.pids[i]) < 0) {
			fprintf(stderr, "quadcontrol: cannot initialize PID %d\n", i);
			resourceDestroy(quad_common.rcbusLock);
			free(quad_common.scenario);
			free(quad_common.pids);
			return -1;
		}
	}
	/* get boundary values of euler angles from ekf module */
	ekf_boundsGet(&quad_common.pids[pwm_yaw].errBound, &quad_common.pids[pwm_roll].errBound, &quad_common.pids[pwm_pitch].errBound);

	/* MMA initialization */
	if (mma_init(&quadCoeffs) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize mma module\n");
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		free(quad_common.pids);
		return -1;
	}


	/* RC bus initialization */
	if (rcbus_init(PATH_DEV_RC_BUS, rc_typeIbus) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize rcbus using %s\n", PATH_DEV_RC_BUS);
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		free(quad_common.pids);
		return -1;
	}

	/* EKF initialization */
	if (ekf_init() < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize ekf\n");
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		free(quad_common.pids);
		return -1;
	}

	return 0;
}


static void quad_help(const char *progName)
{
	printf("Usage: %s [OPTIONS]\n"
		"\t-c <rc/auto>   :  sets control mode\n"
		"\t-h             :  prints help\n", progName);
}


int main(int argc, char **argv)
{
	int err = EOK, c;
	pid_t pid, ret;
	int status = 0;

	if (argc < 2) {
		quad_help(argv[0]);
		return EXIT_FAILURE;
	}

	while ((c = getopt(argc, argv, "c:h")) != -1 && err == EOK) {
		switch (c) {
			case 'c':
				if (strcmp(optarg, "rc") == 0) {
					quad_common.mode = mode_rc;
					printf("Control RC mode.\n");
				}
				else if (strcmp(optarg, "auto") == 0) {
					quad_common.mode = mode_auto;
					printf("Control AUTO mode.\n");
				}
				else {
					err = -EINVAL;
				}
				break;

			case 'h':
				quad_help(argv[0]);
				return EXIT_SUCCESS;

			default:
				quad_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	if (err < 0) {
		quad_help(argv[0]);
		return EXIT_FAILURE;
	}

	priority(1);

	err = quad_init();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	signal(SIGINT, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	signal(SIGTERM, SIG_IGN);

	pid = fork();
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

		if (rcbus_run(quad_rcbusHandler, 500) < 0) {
			fprintf(stderr, "quadcontrol: cannot run rcbus\n");
			quad_done();
			exit(EXIT_FAILURE);
		}

		if (ekf_run() < 0) {
			fprintf(stderr, "quadcontrol: cannot run ekf\n");
			rcbus_stop();
			quad_done();
			exit(EXIT_FAILURE);
		}


		quad_run();

		/* Stop threads from external libs */
		ekf_stop();
		rcbus_stop();

		quad_done();

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
