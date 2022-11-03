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


/* Parses file from `path` with saved mission scenario. Returns array `scenario` with length equal to `sz` */
extern int config_scenarioRead(const char *path, flight_mode_t **scenario, size_t *sz);


#endif
