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

#define NUMBER_MOTORS 4

#define PWM_MIN_SCALER 100000

struct {
	quad_coeffs_t coeffs;
	FILE *files[NUMBER_MOTORS];
} mma_common;


void mma_control(float palt, float proll, float ppitch, float pyaw)
{
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

		fputc(tmp, mma_common.files[i]);
	}
}


void mma_stop(void)
{
	unsigned int i;

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		fputc(PWM_MIN_SCALER, mma_common.files[i]);
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


/* TODO: hardcoded assumption that motors are connected to PWM0...PWM3
         PWM definitions should be moved to board_config.h */
int mma_init(const quad_coeffs_t *coeffs)
{
	int err;
	unsigned int i;
	char buff[32];

	for (i = 0; i < NUMBER_MOTORS; ++i) {
		err = snprintf(buff, sizeof(buff), "/dev/pwm%u", i);
		if (err >= sizeof(buff)) {
			mma_done();
			return -1;
		}

		mma_common.files[i] = fopen(buff, "w");
		if (mma_common.files[i] == NULL) {
			fprintf(stderr, "mma: cannot open pwm file\n");
			mma_done();
			return -1;
		}
	}

	/* TODO: mma_common.coeffs for future usage */
	memcpy(&mma_common.coeffs, coeffs, sizeof(quad_coeffs_t));

	return 0;
}
