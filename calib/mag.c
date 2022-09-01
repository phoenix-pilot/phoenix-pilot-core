/*
 * Phoenix-Pilot
 *
 * mag.c: drone magnetometer calibration module
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "calib.h"


/* sample data to check integrity */
int cal_magmotWrite(FILE *file)
{
	fprintf(file, "p1 1.0\n");
	fprintf(file, "p2 2.0\n");
	fprintf(file, "p3 3.0\n");
	return 0;
}


/* sample data to check integrity */
int cal_magmotInterpret(const char *valName, float val)
{
	if (strcmp(valName, "p1") == 0 && val == 1.0) {
		printf("p1 good\n");
	}
	else if (strcmp(valName, "p2") == 0 && val == 2.0) {
		printf("p2 good\n");
	}
	else if (strcmp(valName, "p3") == 0 && val == 3.0) {
		printf("p3 good\n");
	}
	else {
		printf("wrong data!\n");
		return -EINVAL;
	}

	return EOK;
}


const char *cal_magmotHelp(void)
{
	return "  Magnetometer vs engine interference calibration\n";
}


static int cal_magmotRun(void)
{
	return 0;
}


static int cal_magmotDone(void)
{
	return 0;
}


static int cal_magmotInit(int argc, const char **argv)
{
	return 0;
}


void __attribute__((constructor)) cal_magmotRegister(void)
{
	static calib_t cal = {
		.name = "magmot",
		.init = cal_magmotInit,
		.run = cal_magmotRun,
		.done = cal_magmotDone,
		.interpret = cal_magmotInterpret,
		.write = cal_magmotWrite,
		.help = cal_magmotHelp
	};
	calib_register(&cal);
}
