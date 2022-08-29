/*
 * Phoenix-RTOS
 *
 * MMA (Motor Mixing Algorithm)
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


#define NUMBER_MOTORS 4

#define PWM_MIN_SCALER 100000


const char *motorPaths[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};


struct {
	unsigned int armed;
	handle_t lock;
	quad_coeffs_t coeffs;
} mma_common;


int mma_control(float palt, float proll, float ppitch, float pyaw)
{
	unsigned int i;
	float pwm[NUMBER_MOTORS];

	pwm[0] = palt + proll + ppitch + pyaw;
	pwm[1] = palt - proll - ppitch + pyaw;
	pwm[2] = palt + proll - ppitch - pyaw;
	pwm[3] = palt - proll + ppitch - pyaw;

	mutexLock(mma_common.lock);

	if (!mctl_isArmed()) {
		mutexUnlock(mma_common.lock);

		fprintf(stderr, "mma: cannot set PWMs, module is disarmed\n");
		return -1;
	}

	for (i = 0; i < NUMBER_MOTORS; ++i) {
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

	DEBUG_LOG("PWM: %f, %f, %f, %f\n", pwm[0], pwm[1], pwm[2], pwm[3]);

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


int mma_init(const quad_coeffs_t *coeffs)
{
	int err = 0;

	if ((err = mutexCreate(&mma_common.lock)) < 0) {
		printf("mma: cannot initialize mutex\n");
		return err;
	}

	if (mctl_init(NUMBER_MOTORS, motorPaths) < 0) {
		printf("mma: cannot initialize motors\n");
		resourceDestroy(mma_common.lock);
		return EXIT_FAILURE;
	}

	/* TODO: mma_common.coeffs for future usage */
	memcpy(&mma_common.coeffs, coeffs, sizeof(quad_coeffs_t));

	return 0;
}
