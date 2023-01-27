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
#include "log.h"

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
#include <calib.h>


#define NUMBER_MOTORS 4

#define PID_ATTEN_FACTOR_MAX 2.f
#define PID_ATTEN_FACTOR_MIN 0.f
#define PID_ATTEN_MIDDLE_MAX 0.9f
#define PID_ATTEN_MIDDLE_MIN 0.1f


static const char *motorPaths[] = {
	PWM_MOTOR1, /* path to front left motor */
	PWM_MOTOR2, /* path to rear right motor */
	PWM_MOTOR4, /* path to rear left motor */
	PWM_MOTOR3  /* path to front right motor*/
};


struct {
	handle_t lock;
	quad_coeffs_t coeffs;

	mma_atten_t atten;
	calib_data_t calib;
} mma_common;


static inline void mma_calib(float *val, int motor)
{
	*val = *val * mma_common.calib.params.motlin.motorEq[motor][0] + mma_common.calib.params.motlin.motorEq[motor][1];
}


static void mma_pidAtten(mma_atten_t *atten, float throttle, float *val)
{
	if (throttle < atten->midArg) {
		throttle *= mma_common.atten.startVal + throttle * mma_common.atten.slope[0];
	}
	else {
		throttle *= mma_common.atten.midVal + (throttle - mma_common.atten.midVal) * mma_common.atten.slope[1];
	}
}


int mma_control(float palt, float proll, float ppitch, float pyaw)
{
	unsigned int i;
	float pwm[NUMBER_MOTORS];

	mma_pidAtten(&mma_common.atten, palt, &ppitch);
	mma_pidAtten(&mma_common.atten, palt, &proll);

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
		mma_calib(&pwm[i], i);

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
	calib_free(&mma_common.calib);
}


int mma_init(const quad_coeffs_t *coeffs, const mma_atten_t *atten)
{
	int err;

	if (calib_readFile(CALIB_PATH, typeMotlin, &mma_common.calib) != 0) {
		printf("mma: cannot initialize motlin calibration\n");
		return -1;
	}

	if (atten->startVal < PID_ATTEN_FACTOR_MIN || atten->midVal < PID_ATTEN_FACTOR_MIN || atten->endVal < PID_ATTEN_FACTOR_MIN) {
		printf("mma: attenuation curve below %f\n", PID_ATTEN_FACTOR_MIN);
		return -1;
	}
	if (atten->startVal > PID_ATTEN_FACTOR_MAX || atten->midVal > PID_ATTEN_FACTOR_MAX || atten->endVal > PID_ATTEN_FACTOR_MAX) {
		printf("mma: attenuation curve above %f\n", PID_ATTEN_FACTOR_MAX);
		return -1;
	}
	if (atten->midArg < PID_ATTEN_MIDDLE_MIN || atten->midArg > PID_ATTEN_MIDDLE_MAX) {
		printf("mma: illegal attenuation middle point\n");
		return -1;
	}
	mma_common.atten = *atten;
	mma_common.atten.slope[0] = (mma_common.atten.midVal - mma_common.atten.startVal) / mma_common.atten.midArg;
	mma_common.atten.slope[1] = (mma_common.atten.endVal - mma_common.atten.midVal) / (1 - mma_common.atten.midArg);


	err = mutexCreate(&mma_common.lock);
	if (err < 0) {
		printf("mma: cannot initialize mutex\n");
		calib_free(&mma_common.calib);
		return err;
	}

	if (mctl_init(NUMBER_MOTORS, motorPaths) < 0) {
		printf("mma: cannot initialize motors\n");
		resourceDestroy(mma_common.lock);
		calib_free(&mma_common.calib);
		return EXIT_FAILURE;
	}

	/* TODO: mma_common.coeffs for future usage */
	memcpy(&mma_common.coeffs, coeffs, sizeof(quad_coeffs_t));

	return 0;
}
