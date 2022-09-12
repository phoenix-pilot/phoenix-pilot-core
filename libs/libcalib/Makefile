#
# Makefile for calibration application
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# NAME := libcalib
# CALIBS := magmot.c magiron.c
# LOCAL_SRCS := hmap.c common.c $(CALIBS)
# DEPS := libalgeb libsensc libmctl
# include $(static-lib.mk)

COMMON_SRCS := common.c hmap.c

NAME := libcorr
CAL_MAGIRON := magiron/magiron_corr.c magiron/magiron_common.c
CAL_MAGMOT := magmot/magmot_corr.c magmot/magmot_common.c
LOCAL_SRCS := $(COMMON_SRCS) calib.c $(CAL_MAGIRON) $(CAL_MAGMOT)
LOCAL_HEADERS := libcalib.h hmap.h
DEPS := libalgeb
include $(static-lib.mk)


NAME := libcalib
CAL_MAGIRON := magiron/magiron_calib.c magiron/magiron_common.c
CAL_MAGMOT := magmot/magmot_calib.c magmot/magmot_common.c
LOCAL_SRCS := $(COMMON_SRCS) calib.c $(CAL_MAGIRON) $(CAL_MAGMOT)
LOCAL_HEADERS := libcalib.h hmap.h
DEPS := libalgeb libsensc libmctl libcorr
include $(static-lib.mk)
