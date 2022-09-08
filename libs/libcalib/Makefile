#
# Makefile for calibration application
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Magnetometer calibration tool
NAME := libcalib
CALIBS := magmot.c magiron.c
LOCAL_SRCS := hmap.c $(CALIBS)
DEP_LIBS := libsensc libmctl libalgeb
include $(static-lib.mk)
