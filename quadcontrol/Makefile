#
# Makefile for Phoenix-PILOT quadcopter control
#
# Copyright 2022 Phoenix Systems
#

NAME := quad-control
LOCAL_SRCS := control.c pid.c mma.c log.c config.c qmav.c
DEP_LIBS := libekf libalgeb libsensc libmctl libcalib librcbus libparser libhmap
LIBS := libzynq7000pwm libsensors-spi libzynq7000-gpio-msg libspi-msg
include $(binary.mk)
