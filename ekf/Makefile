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
DEPS := libalgeb libsensc libcalib libparser libhmap libgettime

# Additional compilation flag for pthreads is needed if host target is compiling
ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_LDFLAGS := -lpthread
endif

include $(static-lib.mk)
