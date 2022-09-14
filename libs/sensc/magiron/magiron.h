/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against hard/soft iron interference
 * Module header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _LIBCALIB_MAGIRON_H_
#define _LIBCALIB_MAGIRON_H_

#include <stdio.h>

#define MAGIRON_NAME "magiron"

#define SOFTCAL_ROWSPAN 3
#define SOFTCAL_COLSPAN 3
#define HARDCAL_ROWSPAN 3
#define HARDCAL_COLSPAN 1


struct {
	/* Calibration parameters */
	matrix_t softCal;
	matrix_t hardCal;

	/* Utility variables */
	float softCalBuf[SOFTCAL_ROWSPAN * SOFTCAL_COLSPAN];
	float hardCalBuf[HARDCAL_ROWSPAN * HARDCAL_COLSPAN];
} magiron_common;


/* Returns pointer to help message about this calibration */
const char *magiron_help(void);


/* Configuration file line interpretter */
int magiron_interpret(const char *name, float val);


/* Prints all stored parameters to file */
int magiron_write(FILE *file);


/* All constructor initializations of this calibration module */
void magiron_preinit(void);

#endif
