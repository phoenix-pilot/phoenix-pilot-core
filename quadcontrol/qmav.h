/*
 * Phoenix-RTOS
 *
 * mavlink communication daemon for quadcontrol
 *
 * Copyright 2024 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef QMAV_H
#define QMAV_H

#include <mavlink.h>


/* Sets the qmav heartbeat status flags. Non-blocking operation */
void qmav_setStatus(uint8_t state, uint8_t mode);


/*
 * Sets the qmav position status buffer. Non-blockin operation.
 * Units:
 * - lat/lon in degrees,
 * - h (height) in meters,
 * - vx/vy/vz in m/s,
 * - yawDeg in degrees.
 */ 
void qmav_setPos(double lat, double lon, float h, float vx, float vy, float vz, float yawDeg);


/* Starts previously initialized qmav thread */
int qmav_run(void);


/* Permanently stops qmav module`s thread */
int qmav_stop(void);


/* Deinitializes qmav module */
void qmav_done(void);


/* Initializes qmav module */
int qmav_init(const char *path, int baudrate);


#endif
