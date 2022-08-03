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

NAME := libvec3d
LOCAL_SRCS = vec.c
include $(static-lib.mk)


NAME := libmatrix
LOCAL_SRCS := matrix.c
include $(static-lib.mk)


NAME := libquat
LOCAL_SRCS = quat.c
DEPS := libvec3d
include $(static-lib.mk)
