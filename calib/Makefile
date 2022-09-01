#
# Makefile for calibration application
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Magnetometer calibration tool
NAME := calibtool
CALIBS := mag.c
LOCAL_SRCS := main.c hmap.c $(CALIBS)
DEP_LIBS := libsensc libmctl libalgeb
include $(binary.mk)
