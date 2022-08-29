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
SRCS := $(wildcard $(LOCAL_DIR)matrix/*.c) $(wildcard $(LOCAL_DIR)vec/*.c) $(wildcard $(LOCAL_DIR)quat/*.c)
LIBS := unity
include $(binary.mk)
