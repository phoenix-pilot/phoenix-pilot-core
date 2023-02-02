/*
 * Phoenix-Pilot
 *
 * mctl.c - motors control module
 * 
 * Control over engines, arming and disarmig procedures. No thread safety imposed.
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/msg.h>

#include <board_config.h>
#include <zynq7000-pwm-msg.h>

#include "mctl.h"


#define THROTTLE_DOWN   0.0    /* default/init/lowest position of throttle */
#define THROTTLE_SCALER 100000 /* base thrtl->pwm scaling factor */
#define PWM_MSG_LEN     7      /* length of PWM message sent to pwm driver files + newline */

typedef struct {
	oid_t oid;        /* oid of pwm file associated with this channel */
	uint8_t mask;     /* mask associated with this pwm channel in pwm driver */
	float fval;       /* floating point value in range 0 to 1 of pwm signal */
	uint32_t *outVal; /* channel's associated element of batch transport table */
} mctl_channel_t;


struct {
	mctl_channel_t *motChannel;          /* initialized pwm channels */
	uint32_t pwm[ZYNQ7000_PWM_CHANNELS]; /* all ZYNQ7000_PWM_CHANNELS data storage for batch writing */
	bool init;                           /* motors initialization flag */
	bool armed;                          /* motors armed/disarmed flag */
	unsigned int mNb;                    /* number of motors */
} mctl_common;


/* throttle tempo predetermined values */
static const float mctl_tempoVals[] = { 0, 0.006f, 0.002f };


/* Converts floating point pwm representation in range [0, 1] into driver understandable value. Doesn't clip the value! */
static inline unsigned int mctl_flt2pwm(float thrtl) {
	return (unsigned int)((thrtl + 1.0f) * (float)THROTTLE_SCALER);
}


static int mctl_motWrite(mctl_channel_t *channel, float thrtl)
{
	if (thrtl > 1.f) {
		thrtl = 1.f;
	}
	else if (thrtl < 0.f) {
		thrtl = 0.f;
	}

	*channel->outVal = mctl_flt2pwm(thrtl);

	if (zynq7000pwm_set(&channel->oid, mctl_common.pwm, channel->mask) < 0) {
		return -1;
	}
	channel->fval = thrtl;

	return 0;
}


static inline int mctl_motOff(mctl_channel_t *channel)
{
	if (channel == NULL) {
		return -1;
	}

	*channel->outVal = 0;

	if (zynq7000pwm_set(&channel->oid, mctl_common.pwm, channel->mask) < 0) {
		return -1;
	}
	channel->fval = 0;

	return 0;
}

static int mctl_charChoice(char expected)
{
	char c;

	c = getchar();
	fflush(stdin);

	if (c != expected) {
		return -1;
	}

	return 0;
}


static inline void mctl_printRed(const char *msg)
{
	fprintf(stdout, "%s%s%s", "\033[1;31m", msg, "\033[0m");
	fflush(stdout);
}


int mctl_thrtlBatchSet(const float *throttles, int n)
{
	int i;
	float thrtl;
	mctl_channel_t *channel;
	uint8_t maskSum;

	if (throttles == NULL || n != mctl_common.mNb) {
		return -1;
	}

	if (!mctl_common.init || !mctl_common.armed) {
		fprintf(stderr, "Motors not prepared!\n");
		return -1;
	}

	maskSum = 0;
	for (i = 0; i < n; i++) {
		channel = &mctl_common.motChannel[i];

		thrtl = throttles[i];
		if (thrtl > 1.f) {
			thrtl = 1.f;
		}
		else if (thrtl < 0.f) {
			thrtl = 0.f;
		}

		mctl_common.pwm[channel->oid.id] = mctl_flt2pwm(thrtl);
		maskSum |= channel->mask;
	}

	/* using oid of the first initialized channel as there is no global/superior one */
	if (zynq7000pwm_set(&mctl_common.motChannel->oid, mctl_common.pwm, maskSum) < 0) {
		return -1;
	}

	for (i = 0; i < n; i++) {
		mctl_common.motChannel[i].fval = throttles[i];
	}

	return 0;
}


int mctl_thrtlSet(unsigned int motorIdx, float targetThrottle, enum thrtlTempo tempo)
{
	float currThrtl, change, uchange, rate;
	unsigned int steps;

	if (!mctl_common.init || !mctl_common.armed) {
		fprintf(stderr, "Motors not prepared!\n");
		return -1;
	}

	if (motorIdx >= mctl_common.mNb) {
		return -1;
	}

	/* shortcut when instant change is required */
	if (tempo != tempoInst) {
		rate = mctl_tempoVals[tempo];

		currThrtl = mctl_common.motChannel[motorIdx].fval;
		change = targetThrottle - currThrtl;
		if (fabs(change) < 0.0001) {
			return 0;
		}

		steps = fabs(change / rate);
		uchange = change / steps;
		for (; steps > 0; steps--) {
			currThrtl += uchange;
			if (mctl_motWrite(&mctl_common.motChannel[motorIdx], currThrtl) < 0) {
				return -1;
			}

			usleep(10 * 1000);
		}
	}

	return mctl_motWrite(&mctl_common.motChannel[motorIdx], targetThrottle);
}


bool mctl_isArmed(void)
{
	return mctl_common.armed;
}


int mctl_disarm(void)
{
	int i;
	bool err = false;

	/* 1) Stop the motors by setting throttle to 0 (motors still are armed after this step) */
	for (i = 0; i < mctl_common.mNb; i++) {
		if (mctl_motWrite(&mctl_common.motChannel[i], 0) < 0) {
			err = true;
		}
	}
	usleep(200 * 1000); /* This sleep ensures that ESCs are not fed with no signal (next step) too quickly */

	/* 2) Disarm motors by disabling PWM generation (motors are disarmed after this step) */
	for (i = 0; i < mctl_common.mNb; i++) {
		if (mctl_motOff(&mctl_common.motChannel[i]) < 0) {
			err = true;
		}
	}

	usleep(1000 * 1000); /* wait one second; ESC time dependencies */

	/* as long, as there is any engine armed, we cannot lower armed flag - safety critical! */
	if (!err) {
		mctl_common.armed = false;
		return 0;
	}

	mctl_common.armed = true;

	return -1;
}


int mctl_arm(enum armMode mode)
{
	unsigned int i;

	if (mctl_common.armed) {
		return 0;
	}

	/* every unsupported mode is treated as safeMode */
	if (mode != armMode_auto) {
		mctl_printRed("Engines are about to be armed!\nEnsure safety! Keep distance from engines!\n");

		fprintf(stdout, "Type [y] to continue, or any other key to abort...\n");
		if (mctl_charChoice('y') != 0) {
			fprintf(stdout, "Aborting\n");
			return -1;
		}
	}

	mctl_printRed("Arming engines... \n");
	for (i = 0; i < mctl_common.mNb; i++) {
		if (mctl_motWrite(&mctl_common.motChannel[i], THROTTLE_DOWN) < 0) {
			fprintf(stderr, "Failed to arm\n");
			return -1;
		}
	}

	sleep(2);
	fprintf(stdout, "Engines armed!\n");

	mctl_common.armed = true;

	return 0;
}


void mctl_deinit(void)
{
	unsigned int rep, flag;

	if (mctl_common.armed) {
		rep = 10;
		/* ensure all engines off; safety critical! */
		do {
			/* mctl_disarm sets correct armed */
			flag = mctl_disarm();
			rep--;

			usleep(1000 * 100);
		} while (flag && rep > 0);
	}

	if (mctl_common.init) {
		mctl_common.init = false;

		free(mctl_common.motChannel);
	}
}


int mctl_init(unsigned int motors, const char **motFiles)
{
	int i;

	if (motors == 0 || motors > ZYNQ7000_PWM_CHANNELS) {
		return -1;
	}
	mctl_common.mNb = motors;

	mctl_common.motChannel = calloc(mctl_common.mNb, sizeof(mctl_channel_t));
	if (mctl_common.motChannel == NULL) {
		printf("mctl: allocating channels error. Errno: %d\n", errno);
		return -1;
	}

	/* Validate all given paths and prepare their masks */
	for (i = 0; i < mctl_common.mNb; i++) {
		if (lookup(motFiles[i], NULL, &mctl_common.motChannel[i].oid) < 0) {
			free(mctl_common.motChannel);
			printf("mctl: cannot lookup %s\n", motFiles[i]);
			return -1;
		}

		mctl_common.motChannel[i].mask = (1 << mctl_common.motChannel[i].oid.id);
		mctl_common.motChannel[i].outVal = &mctl_common.pwm[mctl_common.motChannel[i].oid.id];
		mctl_common.motChannel[i].fval = 0;
	}

	mctl_common.init = true;

	return 0;
}
