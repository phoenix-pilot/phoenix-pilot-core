/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against motor interference
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <board_config.h>
#include <libsensors.h>

#include <sensc.h>
#include <vec.h>
#include <calib.h>

#include "calibtool.h"


struct {
	calib_data_t data;
} accrot_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *accrot_dataGet(void)
{
	return &accrot_common.data;
}


const char *accrot_help(void)
{
	return "Accelerometer initial rotation calibration\n";
}


static int accrot_write(FILE *file)
{
	char base[] = "accq";
	quat_t *q = &accrot_common.data.params.accrot.frameQ;

	fprintf(file, "%s0 %f\n", base, q->a);
	fprintf(file, "%s1 %f\n", base, q->i);
	fprintf(file, "%s2 %f\n", base, q->j);
	fprintf(file, "%s3 %f\n", base, q->k);

	return 0;
}


static int accrot_run(void)
{
	return EOK;
}


static int accrot_done(void)
{
	sensc_deinit();

	return EOK;
}


static int accrot_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, false) < 0) {
		return -ENXIO;
	}

	return EOK;
}


__attribute__((constructor(102))) static void accrot_register(void)
{
	static calib_ops_t cal = {
		.name = ACCROT_TAG,
		.init = accrot_init,
		.run = accrot_run,
		.done = accrot_done,
		.write = accrot_write,
		.help = accrot_help,
		.dataGet = accrot_dataGet
	};

	calib_register(&cal);

	accrot_common.data.type = typeAccrot;
}
