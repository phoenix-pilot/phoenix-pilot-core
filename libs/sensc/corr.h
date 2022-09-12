/*
 * Phoenix-Pilot
 *
 * sensorhub client corrections module
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

#include <libsensors.h>


/* applies known corrections to magnetometer event */
extern int corr_mag(sensor_event_t *magEvt);


/* deinitializes correction thread and correction procedures*/
extern void corr_done(void);


/* initializes correction procedures and thread */
extern int corr_init(void);

#endif
