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

# default path for the programs to be installed in rootfs
DEFAULT_INSTALL_PATH := /usr/bin

ifeq ("$(TARGET)","host-generic-pilot")
	ALL_MAKES := $(shell find algebra -name Makefile)
else
	ALL_MAKES := $(shell find . -mindepth 2 -name Makefile)
endif

# read out all components
include $(ALL_MAKES)

# create generic targets
DEFAULT_COMPONENTS := $(ALL_COMPONENTS)

.PHONY: all install clean
all: $(DEFAULT_COMPONENTS)
install: $(patsubst %,%-install,$(DEFAULT_COMPONENTS))
clean: $(patsubst %,%-clean,$(ALL_COMPONENTS))
