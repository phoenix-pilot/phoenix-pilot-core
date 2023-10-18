/*
 * Phoenix-Pilot
 *
 * Config parsers
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _QUADCONTROL_CONFIG_H_
#define _QUADCONTROL_CONFIG_H_

#include "control.h"
#include "pid.h"
#include "mma.h"


/* Parses file from `path` with saved mission scenario. If succeeded returns 0 and `scenario` is array with length equal to `sz`. In other case returns -1.  */
extern int config_scenarioRead(const char *path, flight_mode_t **scenario, size_t *sz);


/* Parser all PIDs from file defined by `path`. If succeeded returns 0 and `pids` are initialized with values from file. In other case returns -1. */
extern int config_pidRead(const char *path, quad_pids_t *ctx, int *sz);


/* Parser all throttle config structures from file defined by `path`. If succeeded returns 0 and `throttle` is array with length equal to `sz`. In other case returns -1. */
extern int config_throttleRead(const char *path, quad_throttle_t **throttle, int *sz);


/* Parser all attitude config structures from file defined by `path`. If succeeded returns 0 and `attitude` is array with length equal to `sz`. In other case returns -1. */
extern int config_attitudeRead(const char *path, quad_att_t **attitude, int *sz);


/* Parses attenuation config structures from file defined by `path`. If succeeded returns 0 and `atten` is array with length equal to one. In other case returns -1. */
extern int config_attenRead(const char *path, mma_atten_t **atten, int *sz);

#endif
