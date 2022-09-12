/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against hard/soft iron interference
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <board_config.h>
#include <libsensors.h>

#include <matrix.h>
#include <vec.h>

#include "libcalib.h"
#include "magiron.h"


#define CHAR_HARDIRON 'h'
#define CHAR_SOFTIRON 's'

#define SOFTCAL_ROWSPAN 3
#define SOFTCAL_COLSPAN 3
#define HARDCAL_ROWSPAN 3
#define HARDCAL_COLSPAN 1


static int magiron_done(void)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


static int magiron_run(void)
{
	printf("This calibration procedure is not implemented and it returns precalculated values!\n Press enter to continue...\n");
	getchar();
	fflush(stdin);

	return EOK;
}


static int magiron_init(int argc, const char **argv)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


__attribute__((constructor(102))) static void calib_magironRegister(void)
{
	static calib_t cal = {
		.name = MAGIRON_NAME,

		.proc.calib.init = magiron_init,
		.proc.calib.run = magiron_run,
		.proc.calib.done = magiron_done,

		.interpret = magiron_interpret,
		.write = magiron_write,
		.help = magiron_help,
	};

	common_register(&cal);

	magiron_preinit();
}
