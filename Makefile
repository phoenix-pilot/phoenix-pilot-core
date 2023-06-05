#
# Makefile for phoenix pilot core components
#
# Copyright 2022 Phoenix Systems
#
# %LICENSE%
#

include ../phoenix-rtos-build/Makefile.common

.DEFAULT_GOAL := all
CFLAGS += -I$(PROJECT_PATH)/

ALL_MAKES := $(shell find . -mindepth 2 -name Makefile)

# default path for the programs to be installed in rootfs
DEFAULT_INSTALL_PATH := /usr/bin

# read out all components
include $(ALL_MAKES)

ifeq ("$(TARGET_FAMILY)-$(TARGET_SUBFAMILY)","host-generic")
	# create host targets
	DEFAULT_COMPONENTS := algebra_tests
	DEFAULT_COMPONENTS += parser_tests
	DEFAULT_COMPONENTS += devekf
else
	# create generic targets
	DEFAULT_COMPONENTS := $(ALL_COMPONENTS)
endif

.PHONY: all install clean
all: $(DEFAULT_COMPONENTS)
install: $(patsubst %,%-install,$(DEFAULT_COMPONENTS))
clean: $(patsubst %,%-clean,$(ALL_COMPONENTS))
