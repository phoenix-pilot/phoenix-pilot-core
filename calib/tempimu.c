/*
 * Phoenix-Pilot
 *
 * Drone motors linear compensation procedure
 * Calibration of inequalities between engines PWMs that level the drone
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

struct {
	calib_data_t data;
} tempimu_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *tempimu_dataGet(void)
{
	return &tempimu_common.data;
}


static const char *tempimu_help(void)
{
	return "Temperature-IMU calibration\n";
}


static int tempimu_write(FILE *file)
{
	int axis;

	/* reference temperature write */
	fprintf(file, "rt %f\n", tempimu_common.data.params.tempimu.refTemp);

	/* accelerometer coefficients */
	for (axis = 0; axis < 3; axis++) {
		fprintf(file, "a%d %f\n", axis, tempimu_common.data.params.tempimu.alfaAcc[axis]);
	}

	/* gyro coefficients */
	for (axis = 0; axis < 3; axis++) {
		fprintf(file, "g%d %f\n", axis, tempimu_common.data.params.tempimu.alfaGyr[axis]);
	}

	return EOK;
}


static int tempimu_done(void)
{
	return EOK;
}


int tempimu_run(void)
{
	printf("This calibration returns default parameters if there are none in calibration file\n");
	printf("Calibration done!");

	return EOK;
}


static int tempimu_init(int argc, const char **argv)
{
	return EOK;
}


__attribute__((constructor(102))) static void tempimu_register(void)
{
	static calib_ops_t cal = {
		.name = TEMPIMU_TAG,
		.init = tempimu_init,
		.run = tempimu_run,
		.done = tempimu_done,
		.write = tempimu_write,
		.help = tempimu_help,
		.dataGet = tempimu_dataGet
	};

	calib_register(&cal);

	tempimu_common.data.type = typeTempimu;
}
