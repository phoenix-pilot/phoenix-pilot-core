#
# Makefile for ekf algebra libraries:
# 
# libvec3d: 3d vector library
# libmatrix: matrix calculations library  
# libquat: quaternion operations library
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

# NAME := libalgeb
# LOCAL_SRCS = vec.c quat.c matrix.c
# include $(static-lib.mk)

NAME := libvec3d
LOCAL_SRCS = vec.c
LOCAL_HEADERS := vec.h
include $(static-lib.mk)


NAME := libmatrix
LOCAL_SRCS := matrix.c
LOCAL_HEADERS := matrix.h
include $(static-lib.mk)


NAME := libquat
LOCAL_SRCS = quat.c
LOCAL_HEADERS := quat.h
DEPS := libvec3d
include $(static-lib.mk)
