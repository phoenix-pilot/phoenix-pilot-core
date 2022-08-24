#
# Makefile for calibration applications
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Magnetometer calibration tool
NAME := calibtool
CALIBS := acal_rot mcal_mot mcal_static
LOCAL_SRCS := main.c cfg_rw.c $(addsuffix .c, $(addprefix procedures/, $(CALIBS)))
DEP_LIBS := libsensc libmctl
include $(binary.mk)
