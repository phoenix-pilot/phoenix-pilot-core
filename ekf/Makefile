#
# Makefile for kalman filter application for Phoenix Pilot
#
# This source code and Makefile is designed for compilation from _user directory of phoenix-rtos-project
#
# Copyright 2022 Phoenix Systems
#

# Files structures
KALMAN_SRCS := kalman_implem.c kalman_core.c kalman_update_imu.c kalman_update_baro.c kalman_update_gps.c

# EKF library
NAME := libekf
LOCAL_SRCS := ekflib.c $(KALMAN_SRCS) measurement.c
LOCAL_HEADERS := ekflib.h
DEPS := libalgeb libsensc libcalib
include $(static-lib.mk)

# EKF development application
NAME := devekf
LOCAL_SRCS := tests/devekf.c
DEP_LIBS := libekf libalgeb libsensc libcalib
LOCAL_LDFLAGS := -z stack-size=16096 -z noexecstack
include $(binary.mk)

NAME := sensortest
LOCAL_SRCS := tests/sensortest.c
DEP_LIBS := libekf libalgeb libsensc libcalib
include $(binary.mk)

NAME := altlog
LOCAL_SRCS := tests/altlog.c
DEP_LIBS := libekf libalgeb libsensc
include $(binary.mk)
