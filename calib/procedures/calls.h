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

struct {
	struct {
		float dummyVal; /* to be expanded with mMotCalib calibration implementation */
	} mMot;

	struct {
		float dummyVal; /* to be expanded with mStaticCalib calibration implementation */
	} mStatic;

	struct {
		float dummyVal; /* to be expanded with aRotCalib calibration implementation */
	} aRot;
} calib_params;


extern int cal_mMotCalib(void);


extern int cal_mStaticCalib(void);


extern int cal_aRotCalib(void);


#endif
