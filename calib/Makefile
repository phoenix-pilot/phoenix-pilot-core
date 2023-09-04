#
# Makefile for calibration application
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Magnetometer calibration tool
NAME := calibtool
CALIBS := magmot.c magiron.c motesc.c motlin.c accorth.c tempimu.c
LOCAL_SRCS := main.c ellcal.c linlsm.c $(CALIBS)
DEP_LIBS := libsensc libmctl libalgeb libcalib libhmap
LIBS := libzynq7000pwm
include $(binary.mk)
