/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against motor interference
 * Module header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _LIBCALIB_MAGMOT_H_
#define _LIBCALIB_MAGMOT_H_

#include <stdio.h>

#include <board_config.h>


#define AVG_SAMPLES  100
#define AVG_WAIT     (1000 * 10)
#define CALIB_POINTS 10

/* FIXME: this should be handled inside, or taken from mtcl */
#define NUM_OF_MOTORS 4
#define PWM_PRESCALER 100000

#define MAGMOT_NAME "magmot"

static const char *motorFiles[] = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};

struct {
	/* motorEq[motorId 0/1/2...NUM_OF_MOTORS][axisId x/y/z][equation_param a/b/c] */
	float motorEq[NUM_OF_MOTORS][3][3];

	/* Correction variables */
	FILE *pwmFiles[NUM_OF_MOTORS];
	vec_t corr;
	handle_t corrLock;
} magmot_common;


/* Returns pointer to help message about this calibration */
const char *magmot_help(void);


/* Configuration file line interpreter */
int magmot_interpret(const char *valName, float val);


/* Prints all stored parameters to file */
int magmot_write(FILE *file);


/* All constructor initializations of this calibration module */
void magmot_preinit(void);

#endif
