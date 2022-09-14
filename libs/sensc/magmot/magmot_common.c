/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against motor interference
 * Common submodule
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
#include <stdlib.h>
#include <errno.h>
#include <sys/threads.h>

#include <board_config.h>
#include <libsensors.h>

#include <sensc.h>
#include <mctl.h>
#include <vec.h>
#include <matrix.h>

#include "../calibcore.h"
#include "magmot.h"

/*
* Writes proper name for calibration parameter of:
* - 'paramId' equation parameter for...
* - 'axisId' magnetometer axis...
* - of 'motorId' motor interference impact in relation to its throttle value.
*
* 'buf' must be allocated and sizeof(buf) = 5.
*/
static void magmot_paramName(unsigned int motorId, unsigned int axisId, unsigned int paramId, char *buf)
{
	char xyz[3] = "xyz";
	char abc[3] = "abc";

	sprintf(buf, "m%i%c%c", motorId, xyz[axisId], abc[paramId]);
}


/* Returns pointer to correct parameter variable given name `paramName` */
static float *magmot_paramSlot(const char *paramName)
{
	unsigned int motor, axis, param;

	if (strlen(paramName) != 4) {
		return NULL;
	}

	/* variable casting for MISRA compliance */
	motor = (uint8_t)(paramName[1] - '0'); /* get motor id */
	axis = (uint8_t)(paramName[2] - 'x');  /* get x/y/z index, knowing that x/y/z are consecutive in ASCII */
	axis = (uint8_t)(paramName[3] - 'a');  /* get a/b/c index, knowing that a/b/c are consecutive in ASCII */

	if (motor >= NUM_OF_MOTORS || axis >= 3 || param >= 3) {
		return NULL;
	}

	return &magmot_common.motorEq[motor][axis][param];
}


int magmot_write(FILE *file)
{
	unsigned int motor, axis, param;
	char paramName[5];

	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		for (axis = 0; axis < 3; axis++) {
			for (param = 0; param < 3; param++) {
				magmot_paramName(motor, axis, param, paramName);
				fprintf(file, "%s %f\n", paramName, magmot_common.motorEq[motor][axis][param]);
			}
		}
	}
	return 0;
}


int magmot_interpret(const char *valName, float val)
{
	float *paramSlot;

	/* get parameter slot for name `valName` */
	paramSlot = magmot_paramSlot(valName);

	if (paramSlot == NULL) {
		return -ENOENT;
	}

	*paramSlot = val;

	return EOK;
}


const char *magmot_help(void)
{
	return "Magnetometer vs engine interference calibration\n";
}


void magmot_preinit(void)
{
	unsigned int motor, axis, param;

	/* set all params to neutral to not disrupt the drone with uninitialized garbage */
	for (motor = 0; motor < NUM_OF_MOTORS; motor++) {
		for (axis = 0; axis < 3; axis++) {
			for (param = 0; param < 3; param++) {
				magmot_common.motorEq[motor][axis][param] = 0.0;
			}
		}
	}
}
