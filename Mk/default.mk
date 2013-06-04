#!/bin/make -f

####################################################################################
# Copy this file to /etc/robot/overwrite.mk if it's not already there. That allows
# you to use it over multiple projects
####################################################################################

####################################################################################
# Define your target
####################################################################################

# The target can be a Raspberry PI, a Surveyor or Replicator robot with a Blackfin
# processor, or your host machine which is probably an x86.
#TARGET_PLATFORM=RASPBERRY, BLACKFIN, or HOST
TARGET_PLATFORM=BLACKFIN

# There are currently only two types of middleware targets supported, there is the
# IROBOT middleware from Stuttgart and there is the HDMR middleware from Karlsruhe.
# The IROBOT middleware has nothing with the iRobot / Roomba robot.
TARGET_MIDDLEWARE=HDMR

####################################################################################
# Create specific macro that can be used in the code to decide if the thing is
# running on the robot, or - with some fake input - on a laptop / desktop
####################################################################################

ifeq ($(TARGET_PLATFORM),HOST)
RUNONPC=true
else
RUNONPC=false
endif

####################################################################################
# Specific macros for the academic Replicator robots
####################################################################################
MULTI_CONTROLLER=false

####################################################################################
# Defaults
####################################################################################

COMPILER_PREFIX=
TARGET=

####################################################################################
# Path and prefix to the Blackfin compiler specific to uClinux
####################################################################################

ifeq ($(TARGET_PLATFORM),BLACKFIN)

# See http://docs.blackfin.uclinux.org/doku.php?id=toolchain:executable_file_formats
# There are two compilers for the Blackfin, one expects you to have uclibc on the 
# board, the other one can be used to compile bare-bone applications. They are 
# bfin-linux-uclibc and bfin-uclinux
COMPILER=bfin-linux-uclibc

# Toolchain from:
#  "aptitude install blackfin-toolchain-uclinux blackfin-toolchain-linux-uclibc"
# is by default installed in /opt/uClinux
CROSS_COMPILER_PATH=/opt/uClinux/$(COMPILER)/bin

CROSS_COMPILER_INCLUDE_PATH:=/opt/uClinux/$(COMPILER)/$(COMPILER)/runtime/usr/include

# Where to install the binaries or libraries
RUNTIME_PATH=/data/blackfin/usr

# The following flags are required for the bfin-uclinux compiler
#LDFLAGS += -Wl,-elf2flt 

# Assume for now the Blackfin 561 architecture
CXXFLAGS=-mcpu=bf561
CFLAGS=-mcpu=bf561 

# Special stack-size options
# http://docs.blackfin.uclinux.org/doku.php?id=uclinux-dist:debugging_applications
# default size is 128k (0x20000), we are doubling it to 256k
STACKSIZE=0x40000
CFLAGS += -Wl,--defsym,__stacksize=$(STACKSIZE)
CXXFLAGS += -Wl,--defsym,__stacksize=$(STACKSIZE)

endif

ifeq ($(TARGET_PLATFORM),RASPBERRY)

COMPILER=arm-linux-gnueabihf

CROSS_COMPILER_PATH=/opt/raspberrypi/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin

RUNTIME_PATH=/data/raspberry/usr

#CXXFLAGS+=-Wl,--no-eh-frame-hdr

endif

ifneq ($(TARGET_PLATFORM),HOST)
# COMPILER_PREFIX and TARGET are the same thing used by different Makefiles
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

HDMR_PATH=/home/ahb/hdmrplus_install
SOAP_PATH=/home/ahb/gsoap-2.8.7

MIDDLEWARE_INCLUDES=-I$(HDMR_PATH)/include -I$(SOAP_PATH)/gsoap
MIDDLEWARE_LIBS=-L$(HDMR_PATH)/lib -L$(SOAP_PATH)/gsoap -lirobot_app -lirobot_common -lPeer -lUdata -lpthread

endif

ifeq ($(TARGET_MIDDLEWARE),IROBOT)

IROBOT_PATH=/home/anne/myworkspace/stuttgart/irobot
LIBCAM_PATH=/home/anne/myworkspace/stuttgart/irobot/libcam

MIDDLEWARE_INCLUDES=-I$(IROBOT_PATH)/include -I$(LIBCAM_PATH)/include 
MIDDLEWARE_LIBS=-L$(IROBOT_PATH)/lib -ljpeg

endif

####################################################################################
# Default compilation, assembler options
####################################################################################

CXXFLAGS+=-O3 -Wno-error=unused-function
ASMFLAGS=
DEBUGFLAGS=-O0

CXXINCLUDE += $(MIDDLEWARE_INCLUDES) -I$(CROSS_COMPILER_INCLUDE_PATH) -I$(RUNTIME_PATH)/local/include -I$(RUNTIME_PATH)/include

CXXFLAGS+=$(CXXINCLUDE)

LDFLAGS=$(MIDDLEWARE_LIBS) \
	-L$(RUNTIME_PATH)/lib \
	-L$(RUNTIME_PATH)/local/lib 


# Semaphores / mutexes
LDFLAGS += -lpthread -lv4l2 -lv4lconvert

# Just use normal compiler, uncomment if you want to cross-compile
# This will add a -DRUNONPC flag to gcc or g++ which can subsequently be
# used in the code to distinguish between code for on the robot and on the PC
ifeq ($(RUNONPC),true)
COMPILER_PREFIX=
CFLAGS += -DRUNONPC -Wall
CXXFLAGS += -DRUNONPC -Wall
else
#CFLAGS += -mmulticore -mcoreb
#CXXFLAGS += -mmulticore -mcoreb
#CFLAGS += -mstack-check-l1
#CXXFLAGS += -mstack-check-l1
endif

ifeq ($(MULTI_CONTROLLER),true)
CFLAGS += -DMULTI_CONTROLLER
CXXFLAGS += -DMULTI_CONTROLLER
endif

ifeq ($(MULTI_CONTROLLER),true)
LDFLAGS += -lrt
endif

CC=$(COMPILER_PREFIX)gcc
CXX=$(COMPILER_PREFIX)g++
CSIZE=$(COMPILER_PREFIX)size
ASM=$(COMPILER_PREFIX)as
STRIP=$(COMPILER_PREFIX)strip
NM=$(COMPILER_PREFIX)nm
AR=$(COMPILER_PREFIX)ar

####################################################################################
# Where can the libraries be found?
####################################################################################

# For v4l-utils
DESTDIR=/home/anne/mydata

