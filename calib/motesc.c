/*
 * Phoenix-Pilot
 *
 * Drone motors ESC calibration module
 * Calibration of PWM values received by ESC-s of motors
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
#include <unistd.h>

#include <mctl.h>
#include <board_config.h>

#include "calibtool.h"


/* FIXME: this should be in calibtool.h */
static const char *pwmFiles[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};


static int motesc_charChoice(char expected)
{
	char c;

	c = getchar();
	fflush(stdin);

	if (c != expected) {
		return -1;
	}

	return 0;
}


static inline void motesc_printRed(const char *msg)
{
	fprintf(stdout, "%s%s%s", "\033[1;31m", msg, "\033[0m");
	fflush(stdout);
}


static const char *motesc_help(void)
{
	return "ESC-s calibration for correct receiving of PWMs\n";
}


static int motesc_done(void)
{
	/* disarming done by run() method */
	mctl_deinit();

	return EOK;
}


int motesc_run(void)
{
	int i;

	motesc_printRed("This is ESC calibration procedure\n");
	fprintf(stdout, " 1) It requires props taken off, or overweighted drone\n");
	fprintf(stdout, " 2) In case of error or engines sudden startup spam ENTER key!\n");
	fprintf(stdout, " 3) important messages will be printed ");
	motesc_printRed("in red\n");

	sleep(2);

	fprintf(stdout, "Enter 'y' to continue or any other key to abort\n");
	if (motesc_charChoice('y') != 0) {
		fprintf(stderr, "Aborting...\n");
		return -1;
	}

	/* Ask user to disconnect the battery */
	fprintf(stdout, "Disconnect the battery. Enter 'y' afterwards to continue or any other key to abort\n");
	if (motesc_charChoice('y') != 0) {
		fprintf(stderr, "Aborting...\n");
		return -1;
	}

	if (mctl_arm(armMode_auto) != 0) {
		fprintf(stderr, "Aborting...\n");
		return -1;
	}

	motesc_printRed("Critical section. You are going to be asked to:\n");
	fprintf(stdout, " 1) Connect the battery\n");
	fprintf(stdout, " 2) Hit ENTER when you hear two rapid beep-s\n");
	fprintf(stdout, " 3) Hit ENTER when you hear one long beep\n");

	motesc_printRed("If motors start to suddenly spin spam ENTER at least 5 consecutive times\n");

	sleep(2);

	/* Ask user to disconnect the battery */
	fprintf(stdout, "Enter 'y' to continue or any other key to abort\n");
	if (motesc_charChoice('y') != 0) {
		fprintf(stderr, "Aborting...\n");
		mctl_disarm();
		return -1;
	}

	/* Critical section. Set all engines to full throttle */
	for (i = 0; i < NUM_OF_MOTORS; i++) {
		if (mctl_thrtlSet(i, 1.0, tempoInst) != 0) {
			fprintf(stderr, "Aborting...\n");
			mctl_disarm();
			return -1;
		}
	}

	/* Ask user to continue */
	fprintf(stdout, " 1) Connect the battery\n");
	fprintf(stdout, " 2) Hit enter after you hear two rapid beeps\n");

	/* Wait for first enter to be pressed */
	if (motesc_charChoice('\n') != 0) {
		fprintf(stderr, "Aborting...\n");
		mctl_disarm();
		return -1;
	}

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		if (mctl_thrtlSet(i, 0.0, tempoInst) != 0) {
			fprintf(stderr, "Aborting...\n");
			mctl_disarm();
			return -1;
		}
	}

	fprintf(stdout, " 3) Hit enter after you heard one long beep\n");
	if (motesc_charChoice('\n') != 0) {
		fprintf(stderr, "Aborting...\n");
		mctl_disarm();
		return -1;
	}

	fprintf(stdout, "Calibration successful\n");
	mctl_disarm();

	return 0;
}


static int motesc_init(int argc, const char **argv)
{
	if (mctl_init(NUM_OF_MOTORS, pwmFiles) < 0) {
		return -EACCES;
	}

	/* arming done by run method as its not yet time for arming */

	return EOK;
}


__attribute__((constructor(102))) static void motesc_register(void)
{
	static calib_ops_t cal = {
		.name = "motesc",
		.init = motesc_init,
		.run = motesc_run,
		.done = motesc_done,
		.write = NULL,
		.help = motesc_help,
		.dataGet = NULL
	};

	calib_register(&cal);
}
