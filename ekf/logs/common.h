/*
 * Phoenix-Pilot
 *
 * Common elements of ekf-specific logging
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#ifndef _EKF_LOG_COMMON_
#define _EKF_LOG_COMMON_

#define LOG_TYPES_CNT 4

#define TIME_LOG_LEN       13
#define TIME_LOG_INDICATOR 'T'

#define IMU_LOG_LEN       55
#define IMU_LOG_INDICATOR 'I'

#define GPS_LOG_LEN       85
#define GPS_LOG_INDICATOR 'P'

#define BARO_LOG_LEN       21
#define BARO_LOG_INDICATOR 'B'


#endif
