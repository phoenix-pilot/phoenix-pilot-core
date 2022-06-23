#
# Makefile for kalman filter application for Phoenix Pilot
#
# This source code and Makefile is designed for compilation from _user directory of phoenix-rtos-project
#
# Copyright 2022 Phoenix Systems
#

# Files structures
KALMAN_SRCS := kalman_implem.c kalman_core.c kalman_update_imu.c kalman_update_baro.c kalman_update_gps.c
TOOLS_SRCS := tools/phmatrix.c tools/rotas_dummy.c

# EKF library
NAME := libekf
LOCAL_SRCS := ekflib.c $(KALMAN_SRCS) $(TOOLS_SRCS) measurement.c sensc.c
include $(static-lib.mk)
