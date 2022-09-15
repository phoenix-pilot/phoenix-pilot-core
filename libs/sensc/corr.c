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


struct {
	calib_data_t magmot;
	calib_data_t magiron;
} corr_common;


void corr_done(void)
{
    return;
}


int corr_init(void)
{

    calib_readFile("/etc/calib.conf", typeMagiron, &corr_common.magiron);

    calib_readFile("/etc/calib.conf", typeMagmot, &corr_common.magmot);

    return 0;
}
