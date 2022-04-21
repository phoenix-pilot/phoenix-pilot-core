#
# Makefile for kalman filter application for Phoenix Pilot
# 
# This source code and Makefile is designed for compilation from _user directory of phoenix-rtos-project
#
# Copyright 2022 Phoenix Systems
#

NAME := ekf
LOCAL_SRCS := main.c measurement.c tools/phmatrix.c tools/rotas_dummy.c kalman_inits.c kalman_jacobians.c kalman_predict.c kalman_update.c
LOCAL_HEADERS := tools/phmatrix.h tools/rotas_dummy.h kalman.h
LOCAL_LDFLAGS := -z stack-size=16096 -z noexecstack

include $(binary.mk)
