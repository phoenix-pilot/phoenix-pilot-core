#
# Makefile for ekf algebra libraries:
# 
# libvec3d: 3d vector library
# libmatrix: matrix calculations library  
# libquat: quaternion operations library
# libqrot: quaternion-vectorial operations library
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

# independent libraries
NAME := libvec3d
LOCAL_SRCS = vec.c
include $(static-lib.mk)


NAME := libquat
LOCAL_SRCS = quat.c
include $(static-lib.mk)


NAME := libmatrix
LOCAL_SRCS := matrix.c
include $(static-lib.mk)


#dependent libraries
NAME := libqrot
LOCAL_SRCS = qrot.c
DEPS := libvec3d libquat
include $(static-lib.mk)
