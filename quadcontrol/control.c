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
#include <vec.h>

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
#define PID_NUMBERS      5

#define PATH_SCENARIO_CONFIG "/etc/q_mission.conf"

#define ANGLE_THRESHOLD_LOW  (M_PI / 4) /* low threshold for landing safety */
#define ANGLE_THRESHOLD_HIGH (M_PI / 2) /* high threshold for maneuvers */
#define RAD2DEG              ((float)180.0 / M_PI)
#define DEG2RAD              0.0174532925f
#define EARTH_G              9.80665F /* m/s^2 */

/* rcbus trigger thresholds for manual switches SWA/SWB/SWC/SWD */
#define RC_CHANNEL_THR_HIGH ((95 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) / 100 + MIN_CHANNEL_VALUE) /* high position threshold */
#define RC_CHANNEL_THR_LOW  ((5 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) / 100 + MIN_CHANNEL_VALUE)  /* low position threshold */
#define RC_OVRD_LEVEL    (1 << 0)
#define RC_OVRD_YAW      (1 << 1)
#define RC_OVRD_THROTTLE (1 << 2)

#define ABORT_FRAMES_THRESH 5       /* number of correct abort frames from RC transmitter to initiate abort sequence */
#define RC_ERROR_TIMEOUT    2000000 /* number of microseconds of continuous rc error that causes flight abort */
#define LOG_PERIOD          500     /* drone control loop logs data once per 'LOG_PERIOD' milliseconds */

/* Flight modes magic numbers */
#define QCTRL_HOVER_THRESH_TIME  3000 /* hover time threshold (milliseconds) */
#define QCTRL_HOVER_THRESH_ALT   1000 /* hover altitude threshold */
#define QCTRL_LAND_THRESH_VSPEED 250  /* landing vertical speed threshold */

#define QCTRL_POS_MINDIST_DFLT 2.0f                       /* Default minimum waypoint acceptance distance (meters) */
#define QCTRL_POS_YAW_THRESH   5.f                        /* Maximum distance (meters) from waypoint which allows UAV to fly non-head-first toward it */
#define QCTRL_POS_YAW_UPDT_MIN (QCTRL_POS_YAW_THRESH * 2) /* Minimum distance (meters) from waypoint at which yaw is updated mid-flight */
#define QCTRL_POS_YAW_ACCEPT   (5 * DEG2RAD)              /* Minimum yaw error to accept yaw change */
#define QCTRL_POS_ALT_ACCEPT   1000                       /* Minimum altitude error to accept altitude change */


typedef enum { mode_rc = 0, mode_auto } control_mode_t;

struct {
	quad_pids_t pids;
	mma_atten_t atten;

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

	struct {
		vec_t setPos;
		int32_t setAlt;

		vec_t *setPosPtr;
		int32_t *setAltPtr;
	} lastTarget;

	quad_throttle_t throttle;
	float hoverThrottle;
} quad_common;


enum { pwm_alt = 0, pwm_roll, pwm_pitch, pwm_yaw, pwm_pos, pwm_max };


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
	quad_common.pids.alt.flags = PID_FULL;
	quad_common.pids.pitch.flags = PID_FULL;
	quad_common.pids.roll.flags = PID_FULL;
	quad_common.pids.yaw.flags = PID_FULL;

	quad_common.pids.pos.flags = PID_FULL;
}


static inline void quad_levelAtt(quad_att_t *att)
{
	att->pitch = 0;
	att->roll = 0;
}


/* Returns [-PI, +PI] yaw angle which points to `enuTarget`. On error returns 0 yaw angle. */
static inline float quad_aimAt(const vec_t *enuTarget)
{
	float yaw;

	/* inverted atan2 arguments: coordinates in ENU, yaw derived from NED */
	yaw = atan2(enuTarget->x, enuTarget->y);

	return (isnan(yaw)) ? 0 : yaw;
}


/* Additive attitude `att` control based on target position `setPos` and current position from `measure` */
static void quad_attPos(const vec_t *setPos, const ekf_state_t *measure, float *dPitchOut, float *dRollOut, time_t dt)
{
	const vec_t currPos = { .x = measure->enuX, .y = measure->enuY, .z = 0 };
	const vec_t currVel = { .x = measure->veloX, .y = measure->veloY, .z = 0 };
	const float accMax = tan(ANGLE_THRESHOLD_LOW) * EARTH_G;

	float yawCos, yawSin, dPitch, dRoll, accLen;
	vec_t accBody, accEarth;

	pid_calc3d(&quad_common.pids.pos, setPos, &currPos, &currVel, dt, &accEarth);

	/* Clip to max acceleration */
	accLen = vec_len(&accEarth);
	if (accLen > accMax) {
		vec_times(&accEarth, accMax / accLen);
	}

	yawCos = cos(measure->yaw);
	yawSin = sin(measure->yaw);

	/* Calculating target acceleration vector in ENU body frame using NED yaw */
	accBody.x = accEarth.x * yawCos - accEarth.y * yawSin;
	accBody.y = accEarth.y * yawCos + accEarth.x * yawSin;

	/* body frame ENU.x acceleration positively correlates with roll, and ENU.y negatively correlates with pitch */
	dRoll = atan(accBody.x / EARTH_G);
	dPitch = -atan(accBody.y / EARTH_G);

	/* Limit dRoll and dPitch to ANGLE_THRESHOLD_LOW */
	if (dRoll > ANGLE_THRESHOLD_LOW) {
		dRoll = ANGLE_THRESHOLD_LOW;
	}
	if (dRoll < -ANGLE_THRESHOLD_LOW) {
		dRoll = -ANGLE_THRESHOLD_LOW;
	}

	if (dPitch > ANGLE_THRESHOLD_LOW) {
		dPitch = ANGLE_THRESHOLD_LOW;
	}
	if (dPitch < -ANGLE_THRESHOLD_LOW) {
		dPitch = -ANGLE_THRESHOLD_LOW;
	}

	*dPitchOut = dPitch;
	*dRollOut = dRoll;
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


static void quad_cmdCockpit(const ekf_state_t *measure)
{
	int alt, dst, hdg, vel;

	alt = measure->enuZ;
	dst = sqrt(measure->enuX * measure->enuX + measure->enuY * measure->enuY);
	vel = sqrt(measure->veloX * measure->veloX + measure->veloY * measure->veloY);

	hdg = measure->yaw * 180.f / M_PI;
	hdg += (measure->yaw < 0) ? 360 : 0;

	log_print("ALT %3d DST %3d HDG %3d VEL %2d\n", alt, dst, hdg, vel);
}

/* Stores target altitude and position (if set) as last used value.  */
static void quad_prevTargetSave(const vec_t *setPos, const int32_t *setAlt)
{
	if (setPos != NULL) {
		quad_common.lastTarget.setPos = *setPos;
		quad_common.lastTarget.setPosPtr = &quad_common.lastTarget.setPos;
	}
	else {
		quad_common.lastTarget.setPosPtr = NULL;
	}

	/* TODO: add possibility of last altitude not being set */
	if (setAlt != NULL) {
		quad_common.lastTarget.setAlt = *setAlt;
		quad_common.lastTarget.setAltPtr = &quad_common.lastTarget.setAlt;
	}
	else {
		quad_common.lastTarget.setAltPtr = NULL;
	}
}

/* Loads last set position and altitude to `setPos` and `alt`. If there was none previous value, uses current data from `measure` */
static void quad_prevTargetLoad(vec_t *setPos, int32_t *alt, ekf_state_t *measure)
{
	/* Loading last target position */
	if (quad_common.lastTarget.setPosPtr == NULL) {
		setPos->x = measure->enuX;
		setPos->y = measure->enuY;
	}
	else {
		*setPos = quad_common.lastTarget.setPos;
	}

	/* Load last target altitude */
	if (quad_common.lastTarget.setAltPtr == NULL) {
		*alt = measure->enuZ;
	}
	else {
		*alt = quad_common.lastTarget.setAlt;
	}
}


/*
 * Drone attitude control
 * `throttle` - base throttle for all engines.
 * `alt` - pointer to target altitude. NULL means no altitude control.
 * `setPos` - pointer to target position in ENU coordinates. NULL means no position control.
 * 'att` - pointer to base attitude. Must not be NULL.
 * `measure` - pointer to latest ekf state reading
 */
static int quad_motorsCtrl(float throttle, int32_t *alt, const vec_t *setPos, const quad_att_t *att, const ekf_state_t *measure)
{
	time_t dt, now;
	float dRollPos = 0, dPitchPos = 0;
	float palt, proll, ppitch, pyaw;

	if (fabs(measure->pitch) > ANGLE_THRESHOLD_HIGH || fabs(measure->roll) > ANGLE_THRESHOLD_HIGH) {
		fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors stop.\n", measure->roll, measure->pitch);
		mma_stop();
		return -1;
	}

	now = quad_timeMsGet();

	dt = now - quad_common.lastTime;
	quad_common.lastTime = now;

	quad_cmdCockpit(measure);
	quad_prevTargetSave(setPos, alt);

	if (setPos != NULL) {
		quad_attPos(setPos, measure, &dPitchPos, &dRollPos, dt);
	}

	palt = pid_calc(&quad_common.pids.alt, *alt / 1000.f, measure->enuZ, measure->veloZ, dt);
	proll = pid_calc(&quad_common.pids.roll, att->roll + dRollPos, measure->roll, measure->rollDot, dt);
	ppitch = pid_calc(&quad_common.pids.pitch, att->pitch + dPitchPos, measure->pitch, measure->pitchDot, dt);
	pyaw = pid_calc(&quad_common.pids.yaw, att->yaw, measure->yaw, measure->yawDot, dt);

	if (mma_control(throttle + palt, proll, ppitch, pyaw) < 0) {
		return -1;
	}

	usleep(1000);

	return 0;
}


/* Overrides parameters selected with `flags` with RC values */
static void quad_rcOverride(quad_att_t *att, float *throttle, uint32_t flags)
{
	int32_t rcRoll, rcPitch, rcThrottle, rcYaw;

	mutexLock(quad_common.rcbusLock);
	rcRoll = quad_common.rcChannels[RC_RIGHT_HSTICK_CH];
	rcPitch = quad_common.rcChannels[RC_RIGHT_VSTICK_CH];
	rcYaw = quad_common.rcChannels[RC_LEFT_HSTICK_CH];
	rcThrottle = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
	mutexUnlock(quad_common.rcbusLock);

	/* +-0.5 radian ~= +- 28 degrees */
	if ((flags & RC_OVRD_LEVEL) != 0 && att != NULL) {
		att->roll = (rcRoll - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2)) / 1000.f;
		/* minus in pitch is to compensate for rc transmitter sign: stick up => higher channel value => negative pitch change */
		att->pitch = -(rcPitch - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2)) / 1000.f;
	}

	/* delta yaw in +-500 milliradians range */
	if ((flags & RC_OVRD_YAW) != 0 && att != NULL) {
		att->yaw += (float)(rcYaw - ((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2)) / 1000.f;
	}

	/* throttle override between throttle.min and throttle.max */
	if ((flags & RC_OVRD_THROTTLE) != 0 && throttle != NULL) {
		*throttle = quad_common.throttle.min + (quad_common.throttle.max - quad_common.throttle.min) * ((float)rcThrottle / (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE));
	}
}


static inline bool quad_rcChLow(int32_t ch)
{
	return (ch <= RC_CHANNEL_THR_LOW);
}


static inline bool quad_rcChHgh(int32_t ch)
{
	return (ch >= RC_CHANNEL_THR_HIGH);
}


/* Handling maintenance modes */

static int quad_idle(void)
{
	int16_t swa, swb, swc, swd, stickThrtl;

	while (quad_common.currFlight == flight_idle) {
		mutexLock(quad_common.rcbusLock);
		swa = quad_common.rcChannels[RC_SWA_CH];
		swb = quad_common.rcChannels[RC_SWB_CH];
		swc = quad_common.rcChannels[RC_SWC_CH];
		swd = quad_common.rcChannels[RC_SWD_CH];
		stickThrtl = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
		mutexUnlock(quad_common.rcbusLock);

		if (quad_rcChLow(swa) && quad_rcChLow(swb) && quad_rcChLow(swc) && quad_rcChLow(swd) && quad_rcChLow(stickThrtl)) {
			printf("quad_idle: f_idle->f_disarm\n");
			quad_common.currFlight = flight_disarm;
			break;
		}

		sleep(1);
		printf("quad_idle: idling...\n");
	}

	return 0;
}


static int quad_disarm(void)
{
	const time_t armPosMin = 3000000;

	int16_t swa, swb, swc, swd, stickThrtl, stickYaw;
	time_t currTime, armReqTime = 0;

	while (quad_common.currFlight == flight_disarm) {
		mutexLock(quad_common.rcbusLock);
		swa = quad_common.rcChannels[RC_SWA_CH];
		swb = quad_common.rcChannels[RC_SWB_CH];
		swc = quad_common.rcChannels[RC_SWC_CH];
		swd = quad_common.rcChannels[RC_SWD_CH];
		stickThrtl = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
		stickYaw = quad_common.rcChannels[RC_LEFT_HSTICK_CH];
		mutexUnlock(quad_common.rcbusLock);

		/* assesing default switches configuration */
		if (!quad_rcChLow(swa) || !quad_rcChLow(swb) || !quad_rcChLow(swc) || !quad_rcChLow(swd) || !quad_rcChLow(stickThrtl)) {
			printf("quad_idle: f_disarm->f_idle\n");
			quad_common.currFlight = flight_idle;
			break;
		}

		/* assesing yaw stick in arm position */
		if (quad_rcChHgh(stickYaw)) {
			if (armReqTime != 0) {
				gettime(&currTime, NULL);
				if ((currTime - armReqTime) > armPosMin) {
					/* yaw stick hold in arm position long enough for arm procedure */
					printf("quad_disarm: f_disarm->f_arm\n");
					quad_common.currFlight = flight_arm;
					break;
				}
			}
			else {
				gettime(&armReqTime, NULL);
			}
		}
		else {
			armReqTime = 0;
		}

		sleep(1);
		printf("quad_disarm: idling...\n");
	}

	return 0;
}


static flight_type_t quad_arm(void)
{
	const time_t maxArmTime = 30 * 1000000;

	time_t armBeginTime, currTime;
	int16_t swa, stickThrtl;

	gettime(&currTime, NULL);
	armBeginTime = currTime;

	printf("Use SWA to start scenario, or throttle for manual mode\n");

	while (quad_common.currFlight == flight_arm && (currTime - armBeginTime) < maxArmTime) {
		gettime(&currTime, NULL);

		mutexLock(quad_common.rcbusLock);
		swa = quad_common.rcChannels[RC_SWA_CH];
		stickThrtl = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
		mutexUnlock(quad_common.rcbusLock);

		if (!quad_rcChLow(swa)) {
			printf("quad_arm: scenario\n");
			return flight_arm;
		}
		else if (!quad_rcChLow(stickThrtl)) {
			printf("quad_arm: f_arm->f_manual\n");
			return flight_manual;
		}

		sleep(1);
	}

	printf("quad_arm: inactivity disarm\n");
	return flight_disarm;
}


static inline void quad_stageStart(time_t *stageStart, time_t *now, const char *msg)
{
	if (*stageStart == 0) {
		*stageStart = *now;
		log_enable();
		log_print(msg);
	}
}

/* Handling flight modes */

static int quad_takeoff(const flight_mode_t *mode)
{
	const int32_t hvrAlt = mode->takeoff.alt;
	const int32_t liftAlt = 2000;

	quad_att_t att;
	ekf_state_t measure;
	float throttle = 0;
	time_t now, last, stageStart, stabTimer;
	int32_t setAlt, alt;
	vec_t setPos, *setPosPtr;
	bool modeComplete = false;
	enum takeoffStage { stage_spool, stage_lift, stage_stabilize, stage_hover } stage = stage_spool;

	ekf_stateGet(&measure);
	att.yaw = measure.yaw;
	setPosPtr = NULL;
	throttle = 0;
	stageStart = 0;

	log_enable();
	log_print("TAKEOFF\n", setAlt);

	now = quad_timeMsGet();
	quad_periodLogEnable(now);
	last = now;
	stabTimer = now;

	while (quad_common.currFlight == flight_takeoff && modeComplete == false) {
		/* Setup timing and logging */
		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		/* Setup basic attitude */
		quad_levelAtt(&att);
		quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);

		ekf_stateGet(&measure);
		alt = measure.enuZ * 1000;
		if (setPosPtr == NULL) {
			/* updates setPos only if setPosPtr is not used (position control turned off) */
			setPos = (vec_t) { .x = measure.enuX, .y = measure.enuY, .z = 0 };
		}

		switch (stage) {
			/*
			 * Gentle engines spoolup.
			 * At stage exit pitch/roll I terms are reset to mitigate non-level drone placement.
			 *
			 * Goal: reach non-lifting engines level for 1 second.
			 */
			case stage_spool:
				quad_stageStart(&stageStart, &now, "TAKEOFF - spool\n");
				setAlt = hvrAlt;

				throttle = (now - stageStart > 1000) ? quad_common.throttle.min : throttle + quad_common.throttle.min * (now - last) / 1000.f;

				if (now - stageStart > 2000 || throttle > quad_common.throttle.min) {
					quad_common.pids.roll.flags |= PID_RESET_I;
					quad_common.pids.pitch.flags |= PID_RESET_I;
					stage = stage_lift;
					stageStart = 0;

					setPosPtr = &setPos;
				}

				break;

			/*
			 * Drone lift from the ground maintaining the balance between overshooting and swift ascend.
			 * Hover throttle initial estimation with integrating P term in rate (current P)/1 second
			 * Additional 1%/s of throttle increase to assure faster/steady lift.
			 *
			 * Goal: reach threshold altitude or threshold ascent speed.
			 */
			case stage_lift:
				quad_stageStart(&stageStart, &now, "TAKEOFF - lift\n");
				setAlt = hvrAlt;

				throttle += (quad_common.pids.alt.p.val.scl + 0.01) * (now - last) / 1000.f;
				quad_common.hoverThrottle = throttle;

				if (alt > liftAlt || measure.veloZ > 1000) {
					stage = stage_stabilize;
					stageStart = 0;
					stabTimer = now;
				}
				break;

			/*
			 * Drone coarse stabilization.
			 * Altitude I term is pumped into hover throttle so it has the most potential of being ascend-neutral.
			 *
			 * Goal: maintain altitude within error for threshold time.
			 */
			case stage_stabilize:
				quad_stageStart(&stageStart, &now, "TAKEOFF - stabilize\n");
				setAlt = hvrAlt;

				if (throttle < quad_common.throttle.max) {
					throttle += quad_common.pids.alt.i.val.scl;
					quad_common.pids.alt.i.val.scl = 0;
					quad_common.hoverThrottle = throttle;
				}

				if ((hvrAlt - alt) > QCTRL_HOVER_THRESH_ALT || (hvrAlt - alt) < -QCTRL_HOVER_THRESH_ALT) {
					stabTimer = now;
				}

				/* stage escape */
				if ((now - stabTimer) > QCTRL_HOVER_THRESH_TIME) {
					stage = stage_hover;
					stageStart = 0;
				}

				break;

			/*
			 * Drone fine stabilization.
			 *
			 * Goal: maintain hover for specified amount of time.
			 */
			case stage_hover:
			default:
				quad_stageStart(&stageStart, &now, "TAKEOFF - hover\n");

				setAlt = hvrAlt;

				/* stage escape */
				if (now - stageStart > mode->takeoff.time) {
					modeComplete = true;
				}

				break;
		}

		if (quad_motorsCtrl(throttle, &setAlt, setPosPtr, &att, &measure) < 0) {
			return -1;
		}

		last = now;
	}

	return 0;
}


static int quad_hover(const flight_mode_t *mode)
{
	quad_att_t att = { 0 };
	time_t now, stabTimer, stageStart = 0;
	ekf_state_t measure;
	vec_t setPos, *setPosPtr;
	int32_t setAlt, alt;
	bool modeComplete = false;
	enum hoverStage { stage_travel, stage_maintain } stage = stage_travel;

	log_enable();
	log_print("HOVER - alt: %d time: %lld\n", mode->hover.alt, mode->hover.time);

	ekf_stateGet(&measure);
	att.yaw = measure.yaw;

	/* Use last target if available */
	quad_prevTargetLoad(&setPos, &setAlt, &measure);
	setPosPtr = &setPos;

	now = quad_timeMsGet();
	quad_periodLogEnable(now);
	stabTimer = now;
	stageStart = 0;

	while (modeComplete == false && quad_common.currFlight == flight_hover) {
		/* Setup timing and logging */
		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		ekf_stateGet(&measure);
		alt = measure.enuZ * 1000;
		setAlt = mode->hover.alt;

		/* Setup basic attitude */
		quad_levelAtt(&att);
		att.yaw = measure.yaw;

		switch (stage) {
			/*
			 *  Drone ascend/descent to target altitude.
			 */
			case stage_travel:
				quad_stageStart(&stageStart, &now, "HOVER - travel\n");

				if ((setAlt - alt) > QCTRL_HOVER_THRESH_ALT || (setAlt - alt) < -QCTRL_HOVER_THRESH_ALT) {
					stabTimer = now;
				}

				if (now - stabTimer > QCTRL_HOVER_THRESH_TIME) {
					stage = stage_maintain;
					stageStart = 0;
				}

				break;

			/*
			 *  Drone hovering on target target altitude.
			 */
			case stage_maintain:
			default:
				quad_stageStart(&stageStart, &now, "HOVER - hovering\n");

				if (now - stageStart > mode->hover.time) {
					modeComplete = true;
				}

				break;
		}

		quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);

		if (quad_motorsCtrl(quad_common.hoverThrottle, &setAlt, setPosPtr, &att, &measure) < 0) {
			return -1;
		}
	}

	log_enable();
	log_print("HOVER - alt: %d complete\n", mode->hover.alt);

	return 0;
}


static int quad_flyto(const flight_mode_t *mode)
{
	const vec_t targetENU = { .x = mode->flyto.posEast / 1000.f, .y = mode->flyto.posNorth / 1000.f, .z = 0 };

	vec_t pos, setPos, *setPosPtr, targetDelta;
	int32_t setAlt, alt;
	quad_att_t att = { 0 };
	ekf_state_t measure;
	time_t now, stageStart;
	float targetDist, acceptDist, targetYaw;
	bool modeComplete = false, yawChange = false;
	enum flytoStage { stage_vertical, stage_horizontal, stage_poshold } stage = stage_vertical;

	log_enable();
	log_print("FLYTO - N/E/H: %d/%d/%d\n", mode->flyto.posNorth, mode->flyto.posEast, mode->flyto.alt);

	/* position and altitude updated preemptively to decide on yaw update */
	ekf_stateGet(&measure);
	att.yaw = measure.yaw;
	targetYaw = measure.yaw;
	alt = measure.enuZ * 1000;
	pos.x = measure.enuX;
	pos.y = measure.enuZ;
	pos.z = 0;

	/* Current and target position setup */
	quad_prevTargetLoad(&setPos, &setAlt, &measure);
	setPosPtr = &setPos;
	vec_dif(&targetENU, &setPos, &targetDelta);

	/* Face the target if sufficiently far away */
	if (vec_len(&targetDelta) > QCTRL_POS_YAW_THRESH) {
		yawChange = true;
		targetYaw = quad_aimAt(&targetDelta);
	}

	/* Minimum arrival acceptance distance setup */
	acceptDist = (float)mode->flyto.dist / 1000;
	if (acceptDist < QCTRL_POS_MINDIST_DFLT) {
		acceptDist = QCTRL_POS_MINDIST_DFLT;
	}

	/* timers update */
	now = quad_timeMsGet();
	quad_periodLogEnable(now);
	stageStart = 0;

	while (modeComplete == false && quad_common.currFlight == flight_flyto) {
		/* Setup timing and logging */
		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		ekf_stateGet(&measure);
		alt = measure.enuZ * 1000;
		pos.x = measure.enuX;
		pos.y = measure.enuY;
		pos.z = 0;

		vec_dif(&targetENU, &pos, &targetDelta);
		targetDist = vec_len(&targetDelta);

		switch (stage) {
			/*
			 * Height and yaw correction step.
			 */
			case stage_vertical:
				quad_stageStart(&stageStart, &now, "FLYTO - height\n");
				setAlt = mode->flyto.alt;
				att.yaw = targetYaw;

				/* exit stage only if yaw and height errors are within acceptance limits */
				if (abs(alt - setAlt) < QCTRL_POS_ALT_ACCEPT && fabs(targetYaw - measure.yaw) < QCTRL_POS_YAW_ACCEPT) {
					stageStart = 0;
					stage = stage_horizontal;
				}

				break;

			/*
			 * Position correction step.
			 * Yaw is not updated closer than the limit value
			 */
			case stage_horizontal:
				quad_stageStart(&stageStart, &now, "FLYTO - pos\n");

				/* Yaw is updated only if it was meant to be updated and UAV is above yaw update threshold distance away from waypoint */
				if (targetDist > QCTRL_POS_YAW_UPDT_MIN && yawChange == true) {
					targetYaw = quad_aimAt(&targetDelta);
				}

				setPos = targetENU;
				setAlt = mode->flyto.alt;
				att.yaw = targetYaw;

				if (targetDist < acceptDist) {
					stageStart = 0;
					stage = stage_poshold;
				}

				break;

			/*
			 * Hover step
			 */
			case stage_poshold:
				quad_stageStart(&stageStart, &now, "FLYTO - hold\n");
				setPos = targetENU;
				setAlt = mode->flyto.alt;
				att.yaw = targetYaw;

				if ((now - stageStart) > mode->flyto.time) {
					stageStart = 0;
					modeComplete = true;
				}

				break;
		}

		quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);

		if (quad_motorsCtrl(quad_common.hoverThrottle, &setAlt, setPosPtr, &att, &measure) < 0) {
			return -1;
		}
	}

	return 0;
}


static int quad_landing(const flight_mode_t *mode)
{
	const time_t transTime = 1000;
	const int32_t altSink = ((float)mode->landing.descent / quad_common.pids.alt.r.k);

	ekf_state_t measure;
	quad_att_t att = { 0 };
	int32_t alt, setAlt, rcThrottle;
	time_t now, stageStart, stabTimer;
	vec_t setPos, *setPosPtr;
	float throttle = quad_common.hoverThrottle;
	bool modeComplete = false;
	enum landStage { stage_descent, stage_transition, stage_touchdown, stage_landed, stage_confirm } stage = stage_descent;

	log_enable();
	log_print("LAND - alt: %d\n", mode->landing.alt);

	/* Use last target if available */
	quad_prevTargetLoad(&setPos, &setAlt, &measure);
	setPosPtr = &setPos;

	ekf_stateGet(&measure);
	att.yaw = measure.yaw;

	now = quad_timeMsGet();
	stabTimer = now;
	stageStart = 0;

	while (modeComplete == false && quad_common.currFlight == flight_landing) {
		/* Setup timing and logging */
		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		ekf_stateGet(&measure);
		alt = measure.enuZ * 1000;

		switch (stage) {
			case stage_descent:
				quad_stageStart(&stageStart, &now, "LAND - descent\n");
				setAlt = mode->landing.alt;

				if ((setAlt - alt) > QCTRL_HOVER_THRESH_ALT || (setAlt - alt) < -QCTRL_HOVER_THRESH_ALT) {
					stabTimer = now;
				}

				if (now - stabTimer > QCTRL_HOVER_THRESH_TIME) {
					stage = stage_transition;
					stageStart = 0;
				}

				break;

			case stage_transition:
				quad_stageStart(&stageStart, &now, "LAND - transition\n");

				if (now - stageStart < transTime) {
					setAlt = alt - altSink * (now - stageStart) / (float)transTime;
				}
				else {
					setAlt = alt - altSink;
					stage = stage_touchdown;
					stageStart = 0;
					stabTimer = now;
				}

				break;

			case stage_touchdown:
				quad_stageStart(&stageStart, &now, "LAND - touchdown\n");
				setAlt = alt - altSink;

				/* meet descend speed threshold */
				if (fabs(measure.veloZ) > (QCTRL_LAND_THRESH_VSPEED / 1000.f)) {
					stabTimer = now;
				}

				if (now - stabTimer > mode->landing.timeout) {
					setAlt = alt - altSink;
					stage = stage_landed;
					stageStart = 0;
					stabTimer = now;
				}

				break;

			case stage_landed:
				quad_stageStart(&stageStart, &now, "LAND - DONE\n");
				setAlt = alt - altSink;
				setPosPtr = NULL;
				throttle = quad_common.throttle.min;

				/*
				 * First two seconds of landed stage checks if UAV indeed landed.
				 * If landed, sudden throttle drop should NOT result in vertical speed increase.
				 */
				if ((now - stageStart) < (transTime * 2)) {
					/*
					 * `stabTimer` measures DESTABILIZATION of drone vertical speed.
					 * It is zeroed to `now` if the vertical speed does not cross the negative speed threshold.
					 */
					if (measure.veloZ > (-QCTRL_LAND_THRESH_VSPEED / 1000.f)) {
						stabTimer = now;
					}

					/* destabilisation means we did not land - revert to 'touchdown' */
					if (now - stabTimer > 250) {
						setAlt = alt - altSink;
						setPosPtr = &setPos;
						throttle = quad_common.hoverThrottle;
						stage = stage_touchdown;
					}
				}
				else {
					throttle = quad_common.throttle.min;
					stage = stage_confirm;
				}

				break;

			case stage_confirm:
				quad_stageStart(&stageStart, &now, "LAND - DONE\n");
				setAlt = alt;
				setPosPtr = NULL;
				throttle = quad_common.throttle.min;
				att.yaw = measure.yaw;

				quad_common.pids.yaw.flags |= PID_IGNORE_I | PID_IGNORE_P;
				quad_common.pids.pitch.flags |= PID_IGNORE_I;
				quad_common.pids.roll.flags |= PID_IGNORE_I;

				mutexLock(quad_common.rcbusLock);
				rcThrottle = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
				mutexUnlock(quad_common.rcbusLock);

				log_print("LANDING DETECTED! Set throttle = minimum to end!\n");
				if (quad_rcChLow(rcThrottle)) {
					log_enable();
					log_print("LANDING DONE!\n");
					modeComplete = true;
				}

				break;
		}

		quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);

		if (quad_motorsCtrl(throttle, &setAlt, setPosPtr, &att, &measure) < 0) {
			return -1;
		}
	}

	return 0;
}


/*
* quad_manual allows for manual and semi manual control over drone in STABILIZE and ALTHOLD modes.
* SWC switch changes the drone behaviour:
* LOW = STABILIZE mode
* MED = ALTHOLD at last STABILIZE height
* HIGH = ALTHOLD at last STABILIZE height + 5m
*/
static int quad_manual(void)
{
	const vec_t homePos = { .x = 0, .y = 0, .z = 0 };

	quad_att_t att;
	ekf_state_t measure;
	float throttle = 0;
	time_t now;
	int32_t setAlt, alt, stickSWC, rcThrottle;
	vec_t setPos;
	const vec_t *setPosPtr;
	enum subType { sub_stab, sub_phld, sub_rth } submode;

	/* Initialize yaw and altitude */
	ekf_stateGet(&measure);
	alt = setAlt = measure.enuZ * 1000;
	att.yaw = measure.yaw;

	log_enable();
	log_print("RC Control\n");

	now = quad_timeMsGet();
	while (quad_common.currFlight == flight_manual) {
		ekf_stateGet(&measure);

		now = quad_timeMsGet();
		quad_periodLogEnable(now);

		/* Setup basic attitude */
		quad_levelAtt(&att);

		/* Read values necessary for safety and submode selection */
		mutexLock(quad_common.rcbusLock);
		stickSWC = quad_common.rcChannels[RC_SWC_CH];
		rcThrottle = quad_common.rcChannels[RC_LEFT_VSTICK_CH];
		mutexUnlock(quad_common.rcbusLock);

		quad_common.pids.alt.flags = PID_FULL;

		/* submode selection */
		if (quad_rcChLow(stickSWC) || stickSWC > MAX_CHANNEL_VALUE) {
			submode = sub_stab; /* SWC == LOW (or illegal value) -> stabilize */
		}
		else if (quad_rcChHgh(stickSWC)) {
			submode = sub_rth; /* SWC == HIGH -> Return to Home (RTH) */
		}
		else {
			submode = sub_phld; /* SWC == MID -> poshold */
		}

		switch (submode) {
			case sub_rth:
				setPosPtr = &homePos; /* Position control turned off */
				setAlt = alt;
				quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);
				break;

			case sub_phld:
				setPosPtr = &setPos; /* Position control turned on */
				setAlt = alt;
				quad_common.pids.pos.flags = PID_FULL;
				quad_rcOverride(&att, NULL, RC_OVRD_LEVEL);
				break;

			case sub_stab:
			default:
				att.yaw = measure.yaw;
				alt = setAlt = measure.enuZ * 1000;

				setPosPtr = NULL; /* Position control turned off */

				/* We don`t want altitude pid to affect the hover in stabilize mode */
				quad_common.pids.alt.flags |= PID_IGNORE_P | PID_IGNORE_I | PID_IGNORE_D;
				quad_common.pids.pos.flags |= PID_IGNORE_P | PID_IGNORE_I | PID_IGNORE_D;

				quad_rcOverride(&att, &throttle, RC_OVRD_LEVEL | RC_OVRD_YAW | RC_OVRD_THROTTLE);

				/* Perform low threshold check only if throttle is at minimum (probable landing) in case of drone tipping off */
				if (rcThrottle < 0.05 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) {
					if (fabs(measure.pitch) > ANGLE_THRESHOLD_LOW || fabs(measure.roll) > ANGLE_THRESHOLD_LOW) {
						fprintf(stderr, "Angles over threshold, roll: %f, pitch: %f. Motors stop.\n", measure.roll, measure.pitch);
						mma_stop();
						return -1;
					}
				}
				break;
		}

		/* updates setPos only if setPosPtr is not used (position control turned off) */
		if (setPosPtr == NULL) {
			setPos = (vec_t) { .x = measure.enuX, .y = measure.enuY, .z = 0 };
		}

		if (quad_motorsCtrl(throttle, &setAlt, setPosPtr, &att, &measure) < 0) {
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
				quad_idle();
				break;

			case flight_disarm:
				if (armed != 0) {
					log_print("f_disarm: disarming motors...\n");
					mma_stop();
					armed = 0;
				}
				log_print("f_disarm: idling...\n");
				quad_disarm();
				break;

			case flight_arm:
				log_print("f_arm\n");
				if (armed == 0) {
					log_print("f_arm: arming motors...\n");
					mma_start();
					armed = 1;
				}
				/* TODO: enable buzzer */
				sleep(1);

				quad_common.currFlight = quad_arm();
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

			case flight_flyto:
				log_print("f_flyto\n");
				err = quad_flyto(&quad_common.scenario[i++]);
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


static void quad_rcbusHandler(const rcbus_msg_t *msg, rcbus_err_t err)
{
	/* abort frame counting variable*/
	static unsigned int abortCnt = 0;
	static time_t rcErrTime = 0;

	time_t currTime;

	/* rcbus error signal handling */
	if (err != rc_err_ok) {
		if (rcErrTime == 0) {
			gettime(&rcErrTime, NULL);
		}
		else {
			gettime(&currTime, NULL);
			if (currTime - rcErrTime > RC_ERROR_TIMEOUT) {
				printf("rcerr: f_abort reached\n");
				quad_common.currFlight = flight_manualAbort;
			}
		}
		return;
	}
	rcErrTime = 0;

	if (msg->channelsCnt < RC_CHANNELS_CNT) {
		fprintf(stderr, "quad-control: rcbus supports insufficient number of channels\n");
		return;
	}

	/* Emergency abort: SWD == MAX, throttle = MIN */
	if (quad_rcChHgh(msg->channels[RC_SWD_CH]) && quad_rcChLow(msg->channels[RC_LEFT_VSTICK_CH]) && quad_common.currFlight < flight_manualAbort) {
		abortCnt++;
		printf("rc: f_abort called %i\n", abortCnt);

		if (abortCnt >= ABORT_FRAMES_THRESH) {
			printf("rc: f_abort reached\n");
			quad_common.currFlight = flight_manualAbort;
		}

		return;
	}
	/* Manual Mode: SWA == MIN */
	else if (quad_rcChLow(msg->channels[RC_SWA_CH]) && quad_common.currFlight > flight_arm && quad_common.currFlight < flight_manual) {
		printf("rc: set f_manual\n");
		quad_common.currFlight = flight_manual;
	}

	/* Reset abort counter if new frame does not call for abort */
	abortCnt = 0;

	mutexLock(quad_common.rcbusLock);
	memcpy(quad_common.rcChannels, msg->channels, sizeof(quad_common.rcChannels));
	mutexUnlock(quad_common.rcbusLock);
}


static void quad_done(void)
{
	mma_stop();
	mma_done();
	ekf_done();
	rcbus_done();

	free(quad_common.scenario);

	resourceDestroy(quad_common.rcbusLock);
}


static int quad_config(void)
{
	int res;
	quad_throttle_t *throttleTmp;
	mma_atten_t *attenTmp;
#if TEST_ATTITUDE
	quad_att_t *attTmp;
#endif

	/* Reading PID configs*/
	if (config_pidRead(PATH_QUAD_CONFIG, &quad_common.pids, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse PIDs from %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	if (res != PID_NUMBERS) {
		fprintf(stderr, "quadcontrol: wrong number of PIDs in %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	/* Reading throttle configs*/
	if (config_throttleRead(PATH_QUAD_CONFIG, &throttleTmp, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse throttle from %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	if (res != 1) {
		fprintf(stderr, "quadcontrol: wrong number of throttle configs in %s\n", PATH_QUAD_CONFIG);
		free(throttleTmp);
		return -1;
	}

	quad_common.throttle = *throttleTmp;
	free(throttleTmp);

	/* Reading pid attenuation configs*/
	if (config_attenRead(PATH_QUAD_CONFIG, &attenTmp, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse pid attenuation from %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	if (res != 1) {
		fprintf(stderr, "quadcontrol: wrong number of attenuation configs in %s\n", PATH_QUAD_CONFIG);
		free(attenTmp);
		return -1;
	}

	quad_common.atten = *attenTmp;
	free(attenTmp);

#if TEST_ATTITUDE
	if (config_attitudeRead(PATH_QUAD_CONFIG, &attTmp, &res) != 0) {
		fprintf(stderr, "quadcontrol: cannot parse attitude from %s\n", PATH_QUAD_CONFIG);
		return -1;
	}

	if (res != 1) {
		fprintf(stderr, "quadcontrol: wrong number of attitude configs in %s\n", PATH_QUAD_CONFIG);
		free(attTmp);
		return -1;
	}

	quad_common.targetAtt = *attTmp;
	free(attTmp);
#endif

	if (config_scenarioRead(PATH_SCENARIO_CONFIG, &quad_common.scenario, &quad_common.scenarioSz) != 0) {
		return -1;
	}

	return 0;
}


static int quad_init(void)
{
	int res;

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
	res = 0;
	res |= pid_init(&quad_common.pids.yaw);
	res |= pid_init(&quad_common.pids.pitch);
	res |= pid_init(&quad_common.pids.roll);
	res |= pid_init(&quad_common.pids.alt);
	res |= pid_init(&quad_common.pids.pos);
	if (res != 0) {
		fprintf(stderr, "quadcontrol: cannot initialize PIDs\n");
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		return -1;
	}

	/* get boundary values of euler angles from ekf module */
	ekf_boundsGet(&quad_common.pids.yaw.errBound, &quad_common.pids.pos.errBound, &quad_common.pids.pos.errBound);
	quad_common.hoverThrottle = quad_common.throttle.min;

	/* MMA initialization */
	if (mma_init(&quadCoeffs, &quad_common.atten) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize mma module\n");
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		return -1;
	}


	/* RC bus initialization */
	if (rcbus_init(PATH_DEV_RC_BUS, rc_typeIbus) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize rcbus using %s\n", PATH_DEV_RC_BUS);
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
		return -1;
	}

	/* EKF initialization */
	if (ekf_init(0) < 0) {
		fprintf(stderr, "quadcontrol: cannot initialize ekf\n");
		resourceDestroy(quad_common.rcbusLock);
		free(quad_common.scenario);
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

	/* Default mma initialization. It will be only used as crash warden */
	mma_init(NULL, NULL);

	signal(SIGINT, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	signal(SIGTERM, SIG_IGN);

	pid = fork();
	/* Parent process waits on child and makes clean up  */
	if (pid > 0) {
		do {
			ret = waitpid(pid, &status, 0);
		} while (ret < 0 && errno == EINTR);

		mma_stop();
	}
	/* Child process runs flight scenario and can be terminated */
	else if (pid == 0) {
		signal(SIGINT, SIG_DFL);
		signal(SIGQUIT, SIG_DFL);
		signal(SIGTERM, SIG_DFL);

		/* Take terminal control */
		tcsetpgrp(STDIN_FILENO, getpid());

		if (quad_init() < 0) {
			fprintf(stderr, "quadcontrol: cannot initialize\n");
			exit(EXIT_FAILURE);
		}

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

	mma_done();

	/* Take back terminal control */
	tcsetpgrp(STDIN_FILENO, getpid());


	return EXIT_SUCCESS;
}
