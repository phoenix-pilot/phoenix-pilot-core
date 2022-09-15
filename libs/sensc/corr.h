/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * sensorhub client correction header
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _SENSC_CORR_H_
#define _SENSC_CORR_H_

#include <calib.h>

#include "corr.h"


/* deinitializes correction procedures */
void corr_done(void);


/* initializes all correction procedures. Returns 0 on success */
int corr_init(void);

#endif
