/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * sensorhub client correction functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <calib.h>

#include "corr.h"


#define CALIB_PATH "/etc/calib.conf"

struct {
	calib_data_t magmot;
	calib_data_t magiron;
} corr_common;


void corr_done(void)
{
	calib_free(&corr_common.magiron);
	calib_free(&corr_common.magmot);
}


int corr_init(void)
{
	int magironRet, magmotRet;

	magironRet = calib_readFile(CALIB_PATH, typeMagiron, &corr_common.magiron);
	magmotRet = calib_readFile(CALIB_PATH, typeMagmot, &corr_common.magmot);

	/* error checking */
	if (magironRet != 0 || magmotRet != 0) {
		if (magmotRet == 0) {
			calib_free(&corr_common.magmot);
		}
		if (magironRet == 0) {
			calib_free(&corr_common.magiron);
		}

		return -1;
	}

	return 0;
}
