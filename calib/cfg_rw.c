/*
 * Phoenix-Pilot
 *
 * calib config read/write separation for cleaner main.c
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "procedures/calls.h"

/* Config file read into `calib_common` structure */
int cal_calibsRead(const char *filepath)
{
	return 0;
}

/* Overwriting config file with values from `calib_common` structure */
int cal_calibsWrite(const char *filepath)
{
	return 0;
}
