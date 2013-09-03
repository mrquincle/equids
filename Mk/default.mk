#!/bin/make -f

####################################################################################
# Copy this file to /etc/robot/overwrite.mk if it's not already there. That allows
# you to use it over multiple projects
####################################################################################

# Use a file local.mk to adapt this file, do not change this file itself, except for
# changes that you want to be valid for everybody! Do not commit this local.mk to 
# the repository. The local.mk file is included in the end so it can overwrite
# everything in this file.
SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

# The file paths.mk contains references to the following:
# - RUNTIME_PATH_ROOT, where you want your compiled applications to end up
# - GENERAL_SOFTWARE_PATH, where you installed e.g. uClinux
# - MIDDLEWARE_PATH_ROOT, parent folder if irobot and/or hdmr+
# Overwrite them in paths.local.mk and do not commit that file
include $(SELF_DIR)/paths.mk

####################################################################################
# Define your target
####################################################################################

# The target can be a Raspberry PI, a Surveyor or Replicator robot with a Blackfin
# processor, or your host machine which is probably an x86. This option will also
# be propagated to the compiler, so you can use #ifdef TARGET_PLATFORM==HOST in your
# code to distinguish the robot use case from the one running on your laptop.
#TARGET_PLATFORM=RASPBERRY, BLACKFIN, or HOST
TARGET_PLATFORM?=BLACKFIN
#TARGET_PLATFORM=HOST

# There are currently only two types of middleware targets supported, there is the
# IROBOT middleware from Stuttgart and there is the HDMR middleware from Karlsruhe.
# The IROBOT middleware has nothing with the iRobot / Roomba robot.
#TARGET_MIDDLEWARE=HDMR
TARGET_MIDDLEWARE?=IROBOT

####################################################################################
# Specific macros for the academic Replicator robots
####################################################################################

# ...

####################################################################################
# Defaults, empty
####################################################################################

COMPILER_PREFIX=
TARGET=

####################################################################################
# When just compiling for host
####################################################################################

ifeq ($(TARGET_PLATFORM),HOST)
#$(warning Compiling for host)
endif

####################################################################################
# Path and prefix to the Blackfin compiler specific to uClinux
####################################################################################

ifeq ($(TARGET_PLATFORM),BLACKFIN)

#$(warning Compiling for blackfin)

# See http://docs.blackfin.uclinux.org/doku.php?id=toolchain:executable_file_formats
# There are two compilers for the Blackfin, one expects you to have uclibc on the 
# board, the other one can be used to compile bare-bone applications. They are 
# bfin-linux-uclibc and bfin-uclinux
COMPILER=bfin-linux-uclibc

# Toolchain from:
#  "aptitude install blackfin-toolchain-uclinux blackfin-toolchain-linux-uclibc"
# is by default installed in /opt/uClinux
CROSS_COMPILER_PATH=$(GENERAL_SOFTWARE_PATH)/uClinux/$(COMPILER)/bin
CROSS_COMPILER_INCLUDE_PATH:=$(GENERAL_SOFTWARE_PATH)/uClinux/$(COMPILER)/$(COMPILER)/runtime/usr/include

# Where to install the binaries or libraries
RUNTIME_PATH=$(RUNTIME_PATH_ROOT)/blackfin/usr

# Target the (dual-core) Blackfin 561 architecture
CXXFLAGS=-mcpu=bf561
CFLAGS=-mcpu=bf561 

# Special stack-size options, default size is 128k (0x20000), double it to 256k
# http://docs.blackfin.uclinux.org/doku.php?id=uclinux-dist:debugging_applications
STACKSIZE=0x40000
CFLAGS += -Wl,--defsym,__stacksize=$(STACKSIZE)
CXXFLAGS += -Wl,--defsym,__stacksize=$(STACKSIZE)

endif

####################################################################################
# Path and prefix to the ARM compiler specific to the Raspberry PI
####################################################################################

ifeq ($(TARGET_PLATFORM),RASPBERRY)

#$(warning Compiling for raspberry)

# This is a hardware implemented floating point ARM processor
COMPILER=arm-linux-gnueabihf

# Toolchain can be obtained from Raspberry PI creators themselves
CROSS_COMPILER_PATH=$(GENERAL_SOFTWARE_PATH)/raspberrypi/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin
CROSS_COMPILER_INCLUDE_PATH=$(GENERAL_SOFTWARE_PATH)/raspberrypi/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/include
# Where to install the binaries or libraries
RUNTIME_PATH=$(RUNTIME_PATH_ROOT)/raspberry/usr

endif

####################################################################################
# In case when - indeed - a certain middleware is used
####################################################################################

# Update variable TARGET, update paths, etc.

ifneq ($(TARGET_PLATFORM),HOST)
COMPILER_PREFIX=$(COMPILER)-
TARGET=$(COMPILER_PREFIX)
PATH:=$(PATH):$(CROSS_COMPILER_PATH)
endif

####################################################################################
# In case of middleware, set additional libraries and include path
####################################################################################

MIDDLEWARE_INCLUDES=
MIDDLEWARE_LIBS=

ifeq ($(TARGET_MIDDLEWARE),HDMR)

HDMR_PATH=$(MIDDLEWARE_PATH_ROOT)/hdmrplus_install
SOAP_PATH=$(MIDDLEWARE_PATH_ROOT)/gsoap-2.8.7
WAPI_PATH=/data/blackfin/usr

MIDDLEWARE_INCLUDES=-I$(HDMR_PATH)/include -I$(SOAP_PATH)/gsoap 
MIDDLEWARE_LIBS=-L$(HDMR_PATH)/lib -L$(SOAP_PATH)/gsoap -lirobot_app \
	-lirobot_common -lirobot_wrap -lPeer -lUdata -lpthread

ifeq ($(ZIGBEE),true)
MIDDLEWARE_INCLUDES+=-I$(WAPI_PATH)/include
MIDDLEWARE_LIBS+=-L$(WAPI_PATH)/lib -lWAPI
endif

endif

ifeq ($(TARGET_MIDDLEWARE),IROBOT)

IROBOT_PATH=$(MIDDLEWARE_PATH_ROOT)/irobot
#LIBCAM_PATH=$(MIDDLEWARE_PATH_ROOT)/irobot/libcam

MIDDLEWARE_INCLUDES=-I$(IROBOT_PATH)/include #-I$(LIBCAM_PATH)/include 
MIDDLEWARE_LIBS=-L$(IROBOT_PATH)/lib -lirobot
endif

####################################################################################
# Additional options for all your controllers
####################################################################################

# Use a local jockey.mk in your controller, if you need a library only for that
# controller

CONTROLLER_LIBS=
CONTROLLER_LIBS+=-lpthread
#CONTROLLER_LIBS+=-lv4l2 -lv4lconvert
#CONTROLLER_LIBS+=-ljpeg 

#$(warning Information: We will use for your controller options: \
#	"$(CONTROLLER_LIBS)". Add for example "-lv4l2 -lv4lconvert" or "-lWAPI" to \
#	CONTROLLER_LIBS in default.mk if necessary for all controllers.)

####################################################################################
# Compilation and assembler options, level of warnings, optimisation
####################################################################################

DEBUGFLAGS=-O3
CFLAGS+=$(DEBUGFLAGS) -Wno-error=unused-function
CXXFLAGS+=$(CFLAGS)
ASMFLAGS=

####################################################################################
# Include all paths and flags in the final variables that are used by the user
####################################################################################

CINCLUDES += $(MIDDLEWARE_INCLUDES) -I$(CROSS_COMPILER_INCLUDE_PATH) \
	-I$(RUNTIME_PATH)/local/include -I$(RUNTIME_PATH)/include

CXXINCLUDES += $(CINCLUDES)

CFLAGS+=$(CINCLUDES)
CXXFLAGS+=$(CXXINCLUDES)

LDFLAGS=$(MIDDLEWARE_LIBS) \
	$(CONTROLLER_LIBS) \
	-L$(RUNTIME_PATH)/lib \
	-L$(RUNTIME_PATH)/local/lib 

####################################################################################
# The shorthands for compiler, linker, assembler, etc. Use these in your Makefiles
####################################################################################

CC=$(COMPILER_PREFIX)gcc
CXX=$(COMPILER_PREFIX)g++
CSIZE=$(COMPILER_PREFIX)size
ASM=$(COMPILER_PREFIX)as
STRIP=$(COMPILER_PREFIX)strip
NM=$(COMPILER_PREFIX)nm
AR=$(COMPILER_PREFIX)ar

####################################################################################
# Final destination path
####################################################################################

DESTDIR=$(RUNTIME_PATH_ROOT)

-include $(SELF_DIR)/local.mk
