/*
 * Phoenix-Pilot
 *
 * calib/procedures - calibration procedures
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PILOT_CALIB_CALLS_H__
#define __PILOT_CALIB_CALLS_H__

#include <vec.h>
#include <quat.h>
#include <stdio.h>

typedef struct {
	vec_t a;
	vec_t b;
	vec_t c;
} quadEq3D_t;

struct {
	/* magnetometer interference from motors on common throttle: trueMag[x,y,z] = readMag[x,y,z] + (a[x,y,z] * meanThrtl + b[x,y,z])^2 */
	struct {
		quadEq3D_t motCal[4];
	} mMot;

	struct {
		float dummyVal; /* to be expanded with mStaticCalib calibration implementation */
	} mStatic;

	struct {
		float dummyVal; /* to be expanded with aRotCalib calibration implementation */
	} aRot;
} calibs_common;


extern int cal_mMotCalib(void);

extern int cal_mStaticCalib(void);

extern int cal_aRotCalib(void);

#endif
