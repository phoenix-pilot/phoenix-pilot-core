#
# Makefile for pilot-core utilities
#
# This source code and Makefile is designed for compilation from phoenix-pilot-core
#
# Copyright 2023 Phoenix Systems
#

NAME := libgettime

ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_SRCS := host_time.c
endif

LOCAL_HEADERS := gettime.h
include $(static-lib.mk)
