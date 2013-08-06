#!/bin/make -f

RUNTIME_PATH_ROOT=$(HOME)/mydata

GENERAL_SOFTWARE_PATH=/opt

MIDDLEWARE_PATH_ROOT=$(HOME)

# Overwrite defaults by paths.local.mk
SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
-include $(SELF_DIR)/paths.local.mk
