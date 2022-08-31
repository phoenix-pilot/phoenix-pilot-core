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
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <board_config.h>

#include "mctl.h"


#define THROTTLE_DOWN   0.0    /* default/init/lowest position of throttle */
#define THROTTLE_SCALER 100000 /* base thrtl->pwm scaling factor */
#define PWM_MSG_LEN     7      /* length of PWM message sent to pwm driver files + newline */


struct {
	FILE **pwmFiles;   /* motors pwm files descriptors */
	float *mThrottles; /* motors current throttle value */
	bool init;         /* motors descriptors initialization flag */
	bool armed;        /* motors armed/disarmed flag */
	unsigned int mNb;  /* number of motors */
} mctl_common;


/* throttle tempo predetermined values */
static const float mctl_tempoVals[] = { 0, 0.006f, 0.002f };


static int mctl_motWrite(unsigned int id, float thrtl)
{
	unsigned int thrtlVal;

	if (id >= mctl_common.mNb) {
		return -1;
	}

	if (thrtl > 1.f) {
		thrtl = 1.f;
	}
	else if (thrtl < 0.f) {
		thrtl = 0.f;
	}

	thrtlVal = (unsigned int)((thrtl + 1.0f) * (float)THROTTLE_SCALER);

	/* check for fprintf() fail, or partial success one PWM message write */
	if (fprintf(mctl_common.pwmFiles[id], "%u\n", thrtlVal) < PWM_MSG_LEN) {
		fprintf(stderr, "mctl: cannot set PWM for motor: %d\n", id);
		return -1;
	}
	fflush(mctl_common.pwmFiles[id]);

	mctl_common.mThrottles[id] = thrtl;

	return 0;
}


static inline int mctl_motOff(unsigned int id)
{
	if (id >= mctl_common.mNb) {
		return -1;
	}

	/* check for fprintf() fail, or partial success on one sign write */
	if (fprintf(mctl_common.pwmFiles[id], "0") < 1) {
		return -1;
	}

	return 0;
}


static inline void mctl_printRed(const char *msg)
{
	fprintf(stdout, "%s%s%s", "\033[1;31m", msg, "\033[0m");
	fflush(stdout);
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

		currThrtl = mctl_common.mThrottles[motorIdx];
		change = targetThrottle - currThrtl;
		if (fabs(change) < 0.0001) {
			return 0;
		}

		steps = fabs(change / rate);
		uchange = change / steps;
		for (; steps > 0; steps--) {
			currThrtl += uchange;
			if (mctl_motWrite(motorIdx, currThrtl) < 0) {
				return -1;
			}

			usleep(10 * 1000);
		}
	}

	return mctl_motWrite(motorIdx, targetThrottle);
}


bool mctl_isArmed(void)
{
	return mctl_common.armed;
}


int mctl_disarm(void)
{
	int i;
	bool err = false;

	for (i = 0; i < mctl_common.mNb; i++) {
		if (mctl_motOff(i) < 0) {
			err = true;
		}
	}

	/* as long, as there is any engine armed, we cannot lower armed flag - safety critical! */
	if (!err) {
		mctl_common.armed = false;
		return 0;
	}

	return -1;
}


int mctl_arm(enum armMode mode)
{
	unsigned int i;
	char choice;

	if (mctl_common.armed) {
		return 0;
	}

	/* every unsupported mode is treated as safeMode */
	if (mode != armMode_auto) {
		mctl_printRed("Engines are about to be armed!\nEnsure safety! Keep distance from engines!\n");

		fprintf(stdout, "Type [y] to continue, or any other key to abort...\n");
		choice = getchar();
		fflush(stdin); /* clear buffer from [enter] if any key was passed */

		if (choice != 'y') {
			fprintf(stdout, "Aborting\n");
			return -1;
		}
	}

	mctl_printRed("Arming engines... \n");
	for (i = 0; i < mctl_common.mNb; i++) {
		if (mctl_motWrite(i, THROTTLE_DOWN) < 0) {
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
	unsigned int i, rep, flag;

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

		for (i = 0; i < mctl_common.mNb; i++) {
			if (mctl_common.pwmFiles[i] != NULL) {
				fclose(mctl_common.pwmFiles[i]);
			}
		}

		free(mctl_common.pwmFiles);
		free(mctl_common.mThrottles);
	}
}


int mctl_init(unsigned int motors, const char **motFiles)
{
	int id, cnt;
	bool err;

	if (motors == 0) {
		return -1;
	}
	mctl_common.mNb = motors;

	mctl_common.pwmFiles = calloc(mctl_common.mNb, sizeof(FILE *));
	mctl_common.mThrottles = calloc(mctl_common.mNb, sizeof(float));
	if (mctl_common.pwmFiles == NULL || mctl_common.mThrottles == NULL) {
		free(mctl_common.pwmFiles);
		free(mctl_common.mThrottles);
		return -1;
	}

	err = false;
	for (id = 0; (id < mctl_common.mNb) && (err == false); id++) {
		cnt = 0;

		mctl_common.pwmFiles[id] = fopen(motFiles[id], "r+");
		while (mctl_common.pwmFiles[id] == NULL && !err) {
			usleep(10 * 1000);
			++cnt;

			if (cnt > 10000) {
				fprintf(stderr, "mctl: timeout waiting on %s \n", motFiles[id]);
				err = true;
				break;
			}

			mctl_common.pwmFiles[id] = fopen(motFiles[id], "r+");
		}
	}


	/* handle error at files opening */
	if (err) {
		/* close all files previous to the failed one */
		for (id--; id >= 0; id--) {
			fclose(mctl_common.pwmFiles[id]);
		}

		free(mctl_common.pwmFiles);
		free(mctl_common.mThrottles);

		return -1;
	}

	mctl_common.init = true;

	return 0;
}
