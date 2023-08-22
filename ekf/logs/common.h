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


#include <libsensors.h>


#define LOG_ID_SIZE         sizeof(uint32_t)
#define LOG_IDENTIFIER_SIZE sizeof(char)
#define LOG_TIMESTAMP_SIZE  sizeof(time_t)
#define LOG_PREFIX_SIZE     (LOG_ID_SIZE + LOG_IDENTIFIER_SIZE + LOG_TIMESTAMP_SIZE)

#define LOG_TYPES_CNT 4

#define TIME_LOG_INDICATOR 'T'
#define TIME_LOG_SIZE      LOG_PREFIX_SIZE

#define IMU_LOG_INDICATOR 'I'
#define IMU_LOG_SIZE      (sizeof(accel_data_t) + sizeof(mag_data_t) + sizeof(gyro_data_t) + LOG_PREFIX_SIZE)

#define GPS_LOG_INDICATOR 'P'
#define GPS_LOG_SIZE      (sizeof(gps_data_t) + LOG_PREFIX_SIZE)

#define BARO_LOG_INDICATOR 'B'
#define BARO_LOG_SIZE      (sizeof(baro_data_t) + LOG_PREFIX_SIZE)


#endif
