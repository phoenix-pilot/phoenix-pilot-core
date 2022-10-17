#
# Makefile for parser library tests
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

LOCAL_DIR := $(call my-dir)

TEST_FILES := $(wildcard ${LOCAL_DIR}files/*)
FILES_DIR := ${PREFIX_ROOTFS}usr/test/parser

${shell mkdir -p ${FILES_DIR}}
${shell cp -r ${TEST_FILES} ${FILES_DIR}}

NAME := parser_tests
LOCAL_SRCS := main.c tests.c tools.c
LOCAL_HEADERS := tools.h

LOCAL_CFLAGS := -g

DEP_LIBS := libparser libhmap
LIBS := unity
include $(binary.mk)
