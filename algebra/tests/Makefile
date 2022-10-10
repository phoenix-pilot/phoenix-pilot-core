#
# Makefile for algebra library tests
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

LOCAL_DIR := $(call my-dir)

NAME := algebra_tests
LOCAL_SRCS := main.c
DEP_LIBS := libalgeb

ifeq ("$(TARGET)","host-generic-pilot")
	LOCAL_LDFLAGS += -lm
endif

SRCS += $(wildcard $(LOCAL_DIR)matrix/*.c)
SRCS += $(wildcard $(LOCAL_DIR)vec/*.c)
SRCS += $(wildcard $(LOCAL_DIR)quat/*.c)
SRCS += $(LOCAL_DIR)tools_tests.c
SRCS += $(LOCAL_DIR)tools.c

LIBS := unity
include $(binary.mk)
