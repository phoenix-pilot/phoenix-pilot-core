/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * sensorhub client correction functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdbool.h>

#include <board_config.h>
#include <calib.h>

#include "corr.h"


#define CALIB_PATH    "/etc/calib.conf"
#define NUM_OF_MOTORS 4


static const char *motorFiles[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};

struct {
	calib_data_t magmot;
	calib_data_t magiron;

	/* for magmot correction */
	FILE *pwmFiles[NUM_OF_MOTORS];
} corr_common;


void corr_done(void)
{
	int i;

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		fclose(corr_common.pwmFiles[i]);
	}

	calib_free(&corr_common.magiron);
	calib_free(&corr_common.magmot);
}


int corr_init(void)
{
	int magironRet, magmotRet;
	int i;
	bool err = false;

	/* open pwm files for magmot correction */
	for (i = 0; i < NUM_OF_MOTORS; i++) {
		corr_common.pwmFiles[i] = fopen(motorFiles[i], "r");
		if (corr_common.pwmFiles[i] == NULL) {
			err = true;
			break;
		}
	}

	if (err) {
		i--;
		while (i >= 0) {
			fclose(corr_common.pwmFiles[i]);
		}
		fprintf(stderr, "corr: failed to open motor files\n");
		return -1;
	}

	magironRet = calib_readFile(CALIB_PATH, typeMagiron, &corr_common.magiron);
	magmotRet = calib_readFile(CALIB_PATH, typeMagmot, &corr_common.magmot);

	/* error checking */
	if (magironRet != 0 || magmotRet != 0) {
		if (magmotRet == 0) {
			calib_free(&corr_common.magiron);
		}
		if (magironRet == 0) {
			calib_free(&corr_common.magiron);
		}

		for (i = 0; i < NUM_OF_MOTORS; i++) {
			fclose(corr_common.pwmFiles[i]);
		}
		return -1;
	}

	return 0;
}
