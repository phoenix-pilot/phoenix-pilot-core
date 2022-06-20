#
#Makefile for kalman filter application for Phoenix Pilot
#
#This source code and Makefile is designed for compilation from _user directory of phoenix - rtos - project
#
#Copyright 2022 Phoenix Systems
#

# Development is conducted on imx6ull platform, but final target should be Xylinx Zynq

# Files structures
KALMAN_SRCS := kalman_implem.c kalman_core.c kalman_update_imu.c kalman_update_baro.c kalman_update_gps.c
TOOLS_SRCS := tools/phmatrix.c tools/rotas_dummy.c

# EKF library
NAME := libekf
LOCAL_SRCS := ekflib.c $(KALMAN_SRCS) $(TOOLS_SRCS) measurement.c sensors_client.c
LOCAL_HEADERS := ekflib.h

include $(static-lib.mk)

# EKF testing app
NAME := ekf-tester
LOCAL_SRCS := tester.c $(KALMAN_SRCS) $(TOOLS_SRCS) measurement.c sensors_client.c
LOCAL_LDFLAGS := -z stack-size=16096 -z noexecstack
LIBS := $(SENS_LIB)
#LOCAL_HEADERS := $(KALMAN_HDRS) $(TOOLS_HDRS)

include $(binary.mk)
