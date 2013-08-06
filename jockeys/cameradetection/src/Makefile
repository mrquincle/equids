# Main Makefile

# Expects that CXXFLAGS and LDFLAGS include the middleware paths, be it irobot, or HDMR+

####################################################################################
# Default configuration files
####################################################################################

# Overwrite EQUID_PATH if the env. var. does not exist with a relative path
ifndef $(EQUID_PATH)
	EQUID_PATH:=$(PWD)/../../..
	export EQUID_PATH
endif

# Makefile for default local settings
-include $(EQUID_PATH)/Mk/default.mk

# Optional global makefile overriding (cross)compiler settings etc.
-include /etc/robot/overwrite.mk

# Extend for this jockey with gsl functionality
LDFLAGS += -lgsl -lgslcblas -lm

####################################################################################
# List the directories you want to include from the "bridles" 
####################################################################################

SUBDIRS+=common
SUBDIRS+=camera
SUBDIRS+=camserver
SUBDIRS+=imageproc
SUBDIRS+=eth
SUBDIRS+=main

####################################################################################
# Name of the final binary
####################################################################################

TARGET=cameradetection

####################################################################################
# Content of Makefile
####################################################################################

# Make temporary targets for cleaning and copying
CLEAN_SUBDIRS=$(addsuffix .clean,$(SUBDIRS))
COPY_SUBDIRS=$(addsuffix .copy,$(SUBDIRS))

# Blob for all object files
OBJS=$(wildcard ../obj/*.o)

# Target to build
$(TARGET): check-env all
	$(CXX) $(CXXDEFINE) -o ../bin/$@ $(OBJS) $(CXXFLAGS) $(LDFLAGS) 
	$(STRIP) ../bin/$@
	$(CSIZE) ../bin/$@

# Check the environmental variable EQUID_PATH
check-env:
ifndef EQUID_PATH
	$(warning Warning: EQUID_PATH is undefined.)
endif

# Upload target to robot, strips it
upload: all obj
	$(STRIP) ../bin/$(TARGET)
	#cat ../bin/robotServer|netcat -l -p 7878 

# Default build target
all: clean create-dirs build-subdirs copy-subdirs

# Default clean target
clean: clean-subdirs
	@echo "Cleaning all objects and binaries in parent directory"
	rm -f ../obj/*.o
	rm -f ../bin/$(TARGET)

# Create directories where binaries and objects are stored
create-dirs:
	@echo "Create target directories"
	mkdir -p ../obj
	mkdir -p ../bin

# Collect build, clean, and copy targets
build-subdirs: $(SUBDIRS)
clean-subdirs: $(CLEAN_SUBDIRS)
copy-subdirs: $(COPY_SUBDIRS)

# What to do on make:
$(SUBDIRS):
	@echo "make $@"
	$(MAKE) -C $@

# What to do on make clean:
$(CLEAN_SUBDIRS): %.clean:
	$(MAKE) -C $* clean 

# What to do on make copy:
$(COPY_SUBDIRS): %.copy:
	@echo "Copy objects from $* to \"obj\" directory"
	cp $*/*.o ../obj;

.PHONY: $(TARGET) all $(SUBDIRS) clean clean-subdirs $(CLEAN_SUBDIRS) copy-subdirs $(COPY_SUBDIRS)

