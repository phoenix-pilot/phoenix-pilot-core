/*
 * Phoenix-Pilot
 *
 * mctl.h - motors control module
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
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <board_config.h>

#include "mctl.h"


#define THROTTLE_DOWN   0.0    /* default/init/lowest position of throttle */
#define THROTTLE_SCALER 100000 /* base thrtl->pwm scaling factor */

struct {
	FILE **motorFile;           /* motors pwm files descriptors */
	float *mThrtl;              /* motors current throttle value */
	unsigned int mctlInitFlag;  /* motors descriptors initialization flag */
	unsigned int mctlArmedFlag; /* motors armed/disarmed flag */
	unsigned int mots;          /* number of motors */
} mctl_common;


static int mctl_motWrite(unsigned int motorIdx, float thrtl)
{
	unsigned int thrtlVal;

	if (motorIdx >= mctl_common.mots) {
		return -1;
	}

	if (thrtl > 1.f) {
		thrtl = 1.f;
	}
	else if (thrtl < 0.f) {
		thrtl = 0.f;
	}

	thrtlVal = (unsigned int)((thrtl + 1.0f) * (float)THROTTLE_SCALER);

	if (fprintf(mctl_common.motorFile[motorIdx], "%u\n", thrtlVal) < 0) {
		fprintf(stderr, "mctl: cannot set PWM for motor: %d\n", motorIdx);
		return -1;
	}
	fflush(mctl_common.motorFile[motorIdx]);

	mctl_common.mThrtl[motorIdx] = thrtl;

	return 0;
}


static inline int mctl_motOff(unsigned int motorIdx)
{
	/* if index out of bounds or error during write, return -1 */
	return (motorIdx >= mctl_common.mots || fprintf(mctl_common.motorFile[motorIdx], "0") < sizeof("0") - 1) ? -1 : 0;
}


static inline void mctl_printRed(const char *msg)
{
	printf("%s%s%s", "\033[1;31m", msg, "\033[0m");
	fflush(stdout);
}


int mctl_thrtlSet(unsigned int motorIdx, float targetThrottle, enum thrtlTempo tempo)
{
	float currThrtl, change, uchange, rate;
	unsigned int steps;

	if (!mctl_common.mctlInitFlag || !mctl_common.mctlArmedFlag) {
		printf("Motors not prepared!\n");
	}

	/* shortcut when instant change is required */
	if (tempo != tempoInst) {
		/* using tempo enum to calculate rate of one small change of engine speed */
		switch (tempo) {
			case tempoSlow:
				/* rate of 0.6% per step */
				rate = 0.6 / 100;
				break;
			case tempoHigh:
				/* rate of 0.2% per step */
				rate = 0.2 / 100;
				break;
			default:
				printf("mctl_thrtlSet: unknown motor tempo %d!\n", tempo);
				return -1;
		}

		currThrtl = mctl_common.mThrtl[motorIdx];
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


int mctl_init(unsigned int motors, const char **motFiles)
{
	unsigned int i, err = 0;

	if (motors == 0) {
		return -1;
	}
	mctl_common.mots = motors;

	mctl_common.motorFile = calloc(mctl_common.mots, sizeof(FILE *));
	mctl_common.mThrtl = calloc(mctl_common.mots, sizeof(float));
	if (mctl_common.motorFile == NULL || mctl_common.mThrtl == NULL) {
		free(mctl_common.motorFile);
		free(mctl_common.mThrtl);
		return -1;
	}

	for (i = 0; i < mctl_common.mots; i++) {
		mctl_common.motorFile[i] = fopen(motFiles[i], "r+");
	}

	mctl_common.mctlInitFlag = 1;

	for (i = 0; i < mctl_common.mots; i++) {
		if (mctl_common.motorFile[i] == NULL) {
			printf("Failed at opening %u-th motor descriptor\n", i);
			err = -1;
		}
	}

	if (err != 0) {
		/* deinit via mctl_deinit() as mctlInitFlag is set */
		mctl_deinit();
	}

	return err;
}


int mctl_arm(unsigned int safeMode)
{
	unsigned int i;
	char choice;

	if (mctl_common.mctlArmedFlag) {
		return 0;
	}

	if (safeMode) {
		mctl_printRed("Engines are about to be armed!\nEnsure safety! Keep distance from engines!\n");

		printf("Type [y] to continue, or any other key to abort...\n");
		choice = getchar();
		fflush(stdin); /* clear buffer from [enter] if any key was passed */

		if (choice != 'y') {
			printf("Aborting\n");
			return -1;
		}
	}

	mctl_printRed("Arming engines... \n");
	for (i = 0; i < mctl_common.mots; i++) {
		if (mctl_motWrite(i, THROTTLE_DOWN)) {
			printf("Failed to arm\n");
			return -1;
		}
	}
	sleep(2);
	printf("Engines armed!\n");

	mctl_common.mctlArmedFlag = 1;
	return 0;
}


int mctl_disarm(void)
{
	int i;
	unsigned int err = 0;

	for (i = 0; i < mctl_common.mots; i++) {
		if (mctl_motOff(i) < 0) {
			err = 1;
		}
	}

	/* as long, as there is any engine armed, we cannot lower armed flag - safety critical! */
	if (!err) {
		mctl_common.mctlArmedFlag = 0;
		return 0;
	}
	return -1;
}


void mctl_deinit(void)
{
	unsigned int i, rep, flag;

	if (mctl_common.mctlArmedFlag) {
		rep = 10;
		/* ensure all engines off; safety critical! */
		do {
			/* mctl_disarm sets correct mctlArmedFlag */
			flag = mctl_disarm();
			usleep(1000 * 100);
		} while (flag && rep > 0);
	}

	if (mctl_common.mctlInitFlag) {
		mctl_common.mctlInitFlag = 0;
		for (i = 0; i < mctl_common.mots; i++) {
			if (mctl_common.motorFile[i] != NULL) {
				fclose(mctl_common.motorFile[i]);
			}
		}
		free(mctl_common.motorFile);
		free(mctl_common.mThrtl);
	}
}


unsigned int mctl_isArmed(void)
{
	return mctl_common.mctlArmedFlag;
}
