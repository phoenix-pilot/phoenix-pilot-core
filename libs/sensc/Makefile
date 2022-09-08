#
# Makefile for pilot-core utilities
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#


# TODO: this can be done using makefile wildcards
COMMONS := sensc.c calibcore.c magiron/magiron_common.c magmot/magmot_common.c
CORRS := corr.c magiron/magiron_corr.c magmot/magmot_corr.c
CALIBS := calib.c magiron/magiron_calib.c magmot/magmot_calib.c


# Sensor client library
NAME := libsensc
LOCAL_SRCS := $(COMMONS) $(CORRS)
LOCAL_HEADERS := sensc.h
DEPS := libalgeb libhmap
include $(static-lib.mk)

# Extended sensor client library for calibration purposes
NAME := libcalib
LOCAL_SRCS := $(COMMONS) $(CORRS) $(CALIBS)
LOCAL_HEADERS := sensc.h calibcore.h calib.h
DEPS := libalgeb libhmap libmctl
include $(static-lib.mk)
