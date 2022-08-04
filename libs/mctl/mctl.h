/*
 * Phoenix-Pilot
 *
 * mctl.c - motors control module
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PILOT_MCTL_H__
#define __PILOT_MCTL_H__

/* 
 * throttle change tempo:
 *  - tempoInst = instant change to new value
 *  - tempoSlow = change throttle to new value gradually, slow mode
 *  - tempoHigh = change throttle to new value gradually, fast mode
 */
enum thrtlTempo { tempoInst, tempoSlow, tempoHigh };

/* Initialize engines module WITH 'motors' engines under 'motFiles' paths */
int mctl_init(unsigned int motors, const char **motFiles);

/* Disarm, and deinitialize engine module */
void mctl_deinit(void);

/* Arm engines. safeMode = true: warnings displayed and user consent needed */
int mctl_arm(unsigned int safeMode);

/* disarm engines */
int mctl_disarm(void);

/* returns 1 if motors are armed, 0 otherwise */
unsigned int mctl_isArmed(void);

/* changes engine throttle (in range [0.0, 1.0]) with given tempo */
int mctl_thrtlSet(unsigned int motorIdx, float targetThrottle, enum thrtlTempo tempo);

#endif
