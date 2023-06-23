#
# Makefile for ekflog module tests
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

NAME := ekflog_tests
LOCAL_SRCS := main.c tests.c ../../log.c

ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_LDFLAGS += -lpthread
endif

LIBS := unity
include $(binary.mk)
