/*
 * Phoenix-RTOS
 *
 * MMA (Motor Mixing Algorithm) for plane
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "mma.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/threads.h>

#include <syslog.h>
#include <board_config.h>

#include <mctl.h>


#define NUMBER_PWM_UNITS 5


static const char *pwmPaths[] = {
	PWM_MAIN_MOTOR,
	PWM_LEFT_AILERON,
	PWM_RIGHT_AILERON,
	PWM_ELEVATOR,
	PWM_RUDDER
};


struct {
	handle_t lock;
} mma_common;


int mma_control(float throttle, float proll, float ppitch, float pyaw)
{
	int i;
	float pwm[NUMBER_PWM_UNITS];

	mutexLock(mma_common.lock);

	pwm[0] = throttle; /* Main Motor: throttle */
	pwm[1] = proll;    /* Left Aileron */
	pwm[2] = proll;    /* Right Aileron */
	pwm[3] = ppitch;   /* Elevator */
	pwm[4] = pyaw;     /* Rudder */


	if (!mctl_isArmed()) {
		mutexUnlock(mma_common.lock);

		fprintf(stderr, "mma: cannot set PWMs, module is disarmed\n");
		return -1;
	}

	for (i = 0; i < NUMBER_PWM_UNITS; ++i) {
		if (pwm[i] > 1.0f) {
			pwm[i] = 1.0f;
		}
		else if (pwm[i] < 0) {
			pwm[i] = 0.0f;
		}

		if (mctl_thrtlSet(i, pwm[i], tempoInst) < 0) {
			fprintf(stderr, "mma: cannot set PWM for motor: %u\n", i);
		}
	}

	mutexUnlock(mma_common.lock);

	return 0;
}


void mma_start(void)
{
	mutexLock(mma_common.lock);
	mctl_arm(armMode_auto);
	mutexUnlock(mma_common.lock);
}


void mma_stop(void)
{
	mutexLock(mma_common.lock);
	mctl_disarm();
	mutexUnlock(mma_common.lock);
}


void mma_done(void)
{
	mutexLock(mma_common.lock);
	mctl_deinit();
	mutexUnlock(mma_common.lock);

	resourceDestroy(mma_common.lock);
}


int mma_init(void)
{
	int err;

	err = mutexCreate(&mma_common.lock);
	if (err < 0) {
		printf("mma: cannot initialize mutex\n");
		return err;
	}

	if (mctl_init(NUMBER_PWM_UNITS, pwmPaths) < 0) {
		printf("mma: cannot initialize motors\n");
		resourceDestroy(mma_common.lock);
		return EXIT_FAILURE;
	}

	return 0;
}
