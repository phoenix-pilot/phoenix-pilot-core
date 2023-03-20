#
# Makefile for utility applications in Phoenix-Pilot project
#
# Copyright 2022 Phoenix Systems
#

# EKF development application
NAME := devekf
LOCAL_SRCS := devekf.c
DEP_LIBS := libekf libparser libhmap libalgeb libsensc libcalib
include $(binary.mk)

NAME := sensortest
LOCAL_SRCS := sensortest.c
DEP_LIBS := libekf libparser libhmap libalgeb libsensc libcalib
include $(binary.mk)

NAME := senslog
LOCAL_SRCS := senslog.c
DEP_LIBS := libsensc libcalib libalgeb
include $(binary.mk)

NAME := rclog
LOCAL_SRCS := rclog.c
DEP_LIBS := librcbus
include $(binary.mk)

NAME := lmatest
LOCAL_SRCS := lmatest.c
DEP_LIBS := libalgeb
include $(binary.mk)
