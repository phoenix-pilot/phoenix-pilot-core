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
#include <unistd.h>
#include <sys/threads.h>

#include <syslog.h>
#include <board_config.h>


#define NUMBER_MOTORS 4

#define PWM_MIN_SCALER 100000

struct {
	unsigned int armed;
	handle_t lock;
	quad_coeffs_t coeffs;
	FILE *files[NUMBER_MOTORS];
} mma_common;


static const char *motorsPwm[] = {
	PWM_MOTOR4,
	PWM_MOTOR1,
	PWM_MOTOR3,
	PWM_MOTOR2
};


int mma_control(float palt, float proll, float ppitch, float pyaw)
{
	int err;
	unsigned int i;
	uint32_t tmp;
	float pwm[NUMBER_MOTORS];

	mutexLock(mma_common.lock);
	if (mma_common.armed == 0) {
		fprintf(stderr, "mma: cannot set PWMs, module is disarmed\n");
		mutexUnlock(mma_common.lock);
		return -1;
	}

	pwm[0] = palt - proll + ppitch - pyaw;
	pwm[1] = palt + proll + ppitch + pyaw;
	pwm[2] = palt + proll - ppitch - pyaw;
	pwm[3] = palt - proll - ppitch + pyaw;


	for (i = 0; i < NUMBER_MOTORS; ++i) {
		if (pwm[i] > 1.0f) {
			pwm[i] = 1.0f;
		}
		else if (pwm[i] < 0) {
			pwm[i] = 0.0f;
		}

		tmp = (uint32_t)((pwm[i] + 1.0f) * (float)PWM_MIN_SCALER);

		err = fprintf(mma_common.files[i], "%u\n", tmp);
		if (err < 0) {
			fprintf(stderr, "mma: cannot set PWM for motor: %s\n", motorsPwm[i]);
		}
		fflush(mma_common.files[i]);
	}
	mutexUnlock(mma_common.lock);

	DEBUG_LOG("PWM: %f, %f, %f, %f\n", pwm[0], pwm[1], pwm[2], pwm[3]);

	return 0;
}


static void _mma_motorsIdle(void)
{
	int err;
	unsigned int i;

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		err = fprintf(mma_common.files[i], "%d\n", PWM_MIN_SCALER);
		if (err < 0) {
			fprintf(stderr, "mma: cannot set PWM for motor: %s\n", motorsPwm[i]);
		}
		fflush(mma_common.files[i]);
	}
}


void mma_start(void)
{
	mutexLock(mma_common.lock);
	/* Arm module */
	mma_common.armed = 1;

	_mma_motorsIdle();
	mutexUnlock(mma_common.lock);

	/* Wait for motor initialization */
	sleep(2);
}


void mma_stop(void)
{
	mutexLock(mma_common.lock);
	/* Disarm module */
	mma_common.armed = 0;

	_mma_motorsIdle();
	mutexUnlock(mma_common.lock);
}


void mma_done(void)
{
	unsigned int i;

	mutexLock(mma_common.lock);

	/* make sure that motors are stopped before closing dev files */
	mma_common.armed = 0;
	_mma_motorsIdle();
	usleep(100 * 1000);

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		if (mma_common.files[i] != NULL) {
			fclose(mma_common.files[i]);
		}
	}
	mutexUnlock(mma_common.lock);

	resourceDestroy(mma_common.lock);
}


int mma_init(const quad_coeffs_t *coeffs)
{
	unsigned int i;
	int cnt, err = 0;

	if ((err = mutexCreate(&mma_common.lock)) < 0) {
		return err;
	}

	/* Initialize motors */
	for (i = 0; i < NUMBER_MOTORS; ++i) {
		cnt = 0;

		mma_common.files[i] = fopen(motorsPwm[i], "r+");
		while (mma_common.files[i] == NULL) {
			usleep(10 * 1000);
			++cnt;

			if (cnt > 10000) {
				fprintf(stderr, "mma: timeout waiting on %s \n", motorsPwm[i]);
				mma_done();
				resourceDestroy(mma_common.lock);
				return EXIT_FAILURE;
			}
			mma_common.files[i] = fopen(motorsPwm[i], "r+");
		}
	}

	/* TODO: mma_common.coeffs for future usage */
	memcpy(&mma_common.coeffs, coeffs, sizeof(quad_coeffs_t));

	return 0;
}
