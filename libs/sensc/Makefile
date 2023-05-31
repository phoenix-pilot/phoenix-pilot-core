#
# Makefile for pilot-core utilities
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2022 Phoenix Systems
#

# Sensorhub client library
NAME := libsensc

# Use virtual implementation of sens.h only if secified targets are being built
ifneq (,$(filter $(TARGET),host-generic-pilot armv7a9-zynq7000-vpilot))
  LOCAL_SRCS += vsensc.c
else
  LOCAL_SRCS := sensc.c corr.c
endif

LOCAL_HEADERS := sensc.h
DEPS := libalgeb libcalib
include $(static-lib.mk)
