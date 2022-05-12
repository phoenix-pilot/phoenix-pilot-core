#
#Makefile for kalman filter application for Phoenix Pilot
#
#This source code and Makefile is designed for compilation from _user directory of phoenix - rtos - project
#
#Copyright 2022 Phoenix Systems
#

# Development is conducted on imx6ull platform, but final target should be Xylinx Zynq
SENS_LIB ?= imx6ull-sensi2c

NAME := ekf
LOCAL_SRCS := main.c measurement.c tools/phmatrix.c tools/rotas_dummy.c kalman_inits.c kalman_jacobians.c kalman_predict.c kalman_update_imu.c kalman_update_baro.c
LOCAL_HEADERS := tools/phmatrix.h tools/rotas_dummy.h kalman.h 
LOCAL_LDFLAGS := -z stack-size=16096 -z noexecstack
LIBS := $(SENS_LIB)

include $(binary.mk)
