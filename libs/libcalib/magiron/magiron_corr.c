/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against hard/soft iron interference
 * Corrections submodule
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <stdlib.h>

#include <libsensors.h>

#include <matrix.h>

#include "libcalib.h"
#include "magiron.h"


static int magiron_do(sensor_event_t *evt)
{
	float measBuf[3] = { evt->mag.magX, evt->mag.magY, evt->mag.magZ };
	float corrBuff[3];
	matrix_t meas = { .data = measBuf, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t corr = { .data = corrBuff, .rows = 3, .cols = 1, .transposed = 0 };

	/* precompiled values need to be subtracted, as they are error, not correction */
	matrix_sub(&meas, &magiron_common.hardCal, NULL);
	matrix_prod(&magiron_common.softCal, &meas, &corr);

	evt->mag.magX = *matrix_at(&corr, 0, 0);
	evt->mag.magY = *matrix_at(&corr, 1, 0);
	evt->mag.magZ = *matrix_at(&corr, 2, 0);

	return 0;
}


__attribute__((constructor(102))) static void corr_magironRegister(void)
{
	static calib_t cal = {
		.name = MAGIRON_NAME,

		.proc.corr.init = NULL,
		.proc.corr.recalc = NULL,
		.proc.corr.done = NULL,
		.proc.corr.perform = magiron_do,
		.proc.corr.delay = 0,

		.interpret = magiron_interpret,
		.write = magiron_write,
		.help = magiron_help
	};

	common_register(&cal);

	magiron_preinit();
}
