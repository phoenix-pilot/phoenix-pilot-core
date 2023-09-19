#
# Makefile for EKF test runner for Phoenix Pilot
#
# Copyright 2023 Phoenix Systems
#
# %LICENSE%
#


NAME := ekf_test_runner
LOCAL_SRCS := main.c config_file_handler.c result_check.c ../../logs/reader.c
LIBS := libparser libhmap libalgeb

ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_LDFLAGS += -lm
endif

include $(binary.mk)
