#
# Makefile for algebra library tests
#
# Copyright 2023 Phoenix Systems
#
# %LICENSE%
#

NAME := ekflog_tests
LOCAL_SRCS := main.c tests.c ../reader.c ../writer.c
DEP_LIBS := libplog
LIBS := unity

ifeq ("$(LOG_VOL_CHECK)","TRUE")
	LOCAL_CFLAGS := -DLOG_VOL_CHECK
	LOCAL_SRCS += ../max_logs.c
endif

include $(binary.mk)
