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

#include <syslog.h>
#include <board_config.h>


#define NUMBER_MOTORS 4

#define PWM_MIN_SCALER 100000

struct {
	quad_coeffs_t coeffs;
	FILE *files[NUMBER_MOTORS];
} mma_common;


static const char *motorsPwm[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};


void mma_control(float palt, float proll, float ppitch, float pyaw)
{
	int err;
	unsigned int i;
	uint32_t tmp;
	float pwm[NUMBER_MOTORS];

	pwm[0] = palt + pyaw + ppitch + proll;
	pwm[1] = palt - pyaw + ppitch - proll;
	pwm[2] = palt - pyaw - ppitch + proll;
	pwm[3] = palt + pyaw - ppitch - proll;

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
			fprintf(stderr, "mma: cannot set PWM for motor: %u\n", i + 1);
		}
		fflush(mma_common.files[i]);
	}

	syslog(LOG_INFO, "MMA pwm1: %f, pwm2: %f, pwm3: %f, pwm4: %f\n", pwm[0], pwm[1], pwm[2], pwm[3]);
}


void mma_stop(void)
{
	int err;
	unsigned int i;

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		err = fprintf(mma_common.files[i], "%d\n", PWM_MIN_SCALER);
		if (err < 0) {
			fprintf(stderr, "mma: cannot set PWM for motor: %u\n", i + 1);
		}
		fflush(mma_common.files[i]);
	}
}


void mma_done(void)
{
	unsigned int i;

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		if (mma_common.files[i] != NULL) {
			fclose(mma_common.files[i]);
		}
	}
}


int mma_init(const quad_coeffs_t *coeffs)
{
	unsigned int i;
	int cnt, err = 0;

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
				return EXIT_FAILURE;
			}
			mma_common.files[i] = fopen(motorsPwm[i], "r+");
		}

		err = fprintf(mma_common.files[i], "%d\n", PWM_MIN_SCALER);
		if (err < 0) {
			fprintf(stderr, "mma: cannot set PWM for motor: %u\n", i + 1);
			mma_done();
			return EXIT_FAILURE;
		}
		fflush(mma_common.files[i]);
	}

	/* Wait for motor initialization */
	sleep(2);

	/* TODO: mma_common.coeffs for future usage */
	memcpy(&mma_common.coeffs, coeffs, sizeof(quad_coeffs_t));

	return 0;
}
