#
# Makefile for kalman filter application for Phoenix Pilot
#
# Copyright 2022 Phoenix Systems
#

# Files structures
KALMAN_SRCS := kalman_implem.c kalman_core.c kalman_update_imu.c kalman_update_baro.c kalman_update_gps.c

# EKF library
NAME := libekf
LOCAL_SRCS := ekflib.c $(KALMAN_SRCS) meas.c filters.c log.c
LOCAL_HEADERS := ekflib.h
DEPS := libalgeb libsensc libcalib
include $(static-lib.mk)
