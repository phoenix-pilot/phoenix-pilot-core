/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against motor interference
 * Correction submodule
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
#include <errno.h>
#include <sys/threads.h>

#include <vec.h>
#include <libsensors.h>

#include "../calibcore.h"
#include "magmot.h"
#include "../corr.h"


static int magmot_init(void)
{
	int i = 0;
	bool err = false;

	if (mutexCreate(&magmot_common.corrLock) < 0) {
		return -ENOMEM;
	}

	while (i < NUM_OF_MOTORS && !err) {
		magmot_common.pwmFiles[i] = fopen(motorFiles[i], "r");
		err = (magmot_common.pwmFiles[i] == NULL);
	}

	if (err) {
		/* if error occurred close all previously opened files */
		while (--i >= 0) {
			fclose(magmot_common.pwmFiles[i]);
		}
		resourceDestroy(magmot_common.corrLock);

		return -ENOMEM;
	}

	return EOK;
}


static int magmot_done(void)
{
	int i = 0;

	while (i < NUM_OF_MOTORS) {
		fclose(magmot_common.pwmFiles[i++]);
	}

	resourceDestroy(magmot_common.corrLock);

	return EOK;
}


static int magmot_recalc(void)
{
	long throttles[NUM_OF_MOTORS];
	char buff[16];
	int motor, param;
	float throttle;
	vec_t axisImpact[3], impact = { 0 };

	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		fread(buff, sizeof(char), sizeof(buff), magmot_common.pwmFiles[motor]);

		throttles[motor] = strtod(buff, NULL);

		/* strtod fail also fails this check */
		throttles[motor] -= PWM_PRESCALER;
		if (throttles[motor] > PWM_PRESCALER || throttles[motor] < 0) {

			return -EINVAL;
		}
	}

	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		throttle = (float)throttles[motor] / PWM_PRESCALER;

		for (param = 0; param < 3; param++) {
			axisImpact[param].x = magmot_common.motorEq[motor][0][param];
			axisImpact[param].y = magmot_common.motorEq[motor][1][param];
			axisImpact[param].z = magmot_common.motorEq[motor][2][param];
		}

		vec_times(&axisImpact[0], throttle * throttle);
		vec_times(&axisImpact[1], throttle);

		vec_sub(&impact, &axisImpact[0]);
		vec_sub(&impact, &axisImpact[1]);
		vec_sub(&impact, &axisImpact[2]);
	}

	mutexLock(magmot_common.corrLock);
	magmot_common.corr = impact;
	mutexUnlock(magmot_common.corrLock);

	return EOK;
}


static int magmot_do(sensor_event_t *evt)
{
	mutexLock(magmot_common.corrLock);
	evt->mag.magX += magmot_common.corr.x;
	evt->mag.magY += magmot_common.corr.y;
	evt->mag.magZ += magmot_common.corr.z;
	mutexUnlock(magmot_common.corrLock);

	return EOK;
}


__attribute__((constructor(102))) static void cal_magmotRegister(void)
{

	static calib_t cal = {
		.proc.corr.init = magmot_init,
		.proc.corr.done = magmot_done,
		.proc.corr.recalc = magmot_recalc,
		.proc.corr.perform = magmot_do,
		.proc.corr.delay = 100 * 1000,

		.interpret = magmot_interpret,
		.write = magmot_write,
		.help = magmot_help,
	};

	corr_register(&cal);

	magmot_preinit();
}