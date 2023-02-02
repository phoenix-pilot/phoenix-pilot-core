/*
 * Phoenix-Pilot
 *
 * mctl.h - motors control module
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

#include <stdbool.h>


/* 
 * throttle change tempo:
 *  - tempoInst = instant change to new value
 *  - tempoSlow = change throttle to new value gradually, slow mode
 *  - tempoHigh = change throttle to new value gradually, fast mode
 */
enum thrtlTempo { tempoInst = 0, tempoSlow = 1, tempoHigh = 2 };


enum armMode { armMode_user, armMode_auto };


/* sets values of all initialized throttles to `throttles`. `n` is length of `throttles` */
int mctl_thrtlBatchSet(const float *throttles, int n);


/* changes engine throttle (in range [0.0, 1.0]) with given tempo */
int mctl_thrtlSet(unsigned int motorIdx, float targetThrottle, enum thrtlTempo tempo);


/* returns 1 if motors are armed, 0 otherwise */
bool mctl_isArmed(void);


/* disarm engines */
int mctl_disarm(void);


/* Arm engines. safeMode = true: warnings displayed and user consent needed */
int mctl_arm(enum armMode mode);


/* Disarm, and deinitialize engine module */
void mctl_deinit(void);


/* Initialize engines module WITH 'motors' engines under 'motFiles' paths */
int mctl_init(unsigned int motors, const char **motFiles);

#endif
