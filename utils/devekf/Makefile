#
# Makefile for utility applications in Phoenix-Pilot project
#
# Copyright 2023 Phoenix Systems
#

ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_LDFLAGS += -lm
endif

# EKF development application
NAME := devekf
LOCAL_SRCS := devekf.c
DEP_LIBS := libekf libparser libhmap libalgeb libsensc libcalib libgettime
include $(binary.mk)