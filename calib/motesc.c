/*
 * Phoenix-Pilot
 *
 * Drone motors ESC calibration module
 * Calibration of PWM values received by ESC-s of motors
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>

#include "calibtool.h"


static const char *motesc_help(void)
{
	return "ESC-s calibration for correct receiving of PWMs\n";
}


static int motesc_done(void)
{
	return EOK;
}


static int motesc_run(void)
{
	return EOK;
}


static int motesc_init(int argc, const char **argv)
{
	return EOK;
}


__attribute__((constructor(102))) static void motoresc_register(void)
{
	static calib_ops_t cal = {
		.name = "motoresc",
		.init = motesc_init,
		.run = motesc_run,
		.done = motesc_done,
		.write = NULL,
		.help = motesc_help,
		.dataGet = NULL
	};

	calib_register(&cal);
}