#
# Makefile for calibration application
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Magnetometer calibration tool
NAME := calibtool
CALIBS := magmot.c magiron.c motesc.c motlin.c accrot.c
LOCAL_SRCS := main.c $(CALIBS)
DEP_LIBS := libsensc libmctl libalgeb libcalib hmap
include $(binary.mk)
