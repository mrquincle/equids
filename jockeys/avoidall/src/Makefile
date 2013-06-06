# Main Makefile

# Expects that CXXFLAGS and LDFLAGS include the middleware paths, be it irobot, or HDMR+

####################################################################################
# Default configuration files
####################################################################################

# Makefile for default local settings
-include $(EQUID_PATH)/Mk/default.mk

# Optional global makefile overriding (cross)compiler settings etc.
-include /etc/robot/overwrite.mk

####################################################################################
# List the directories you want to include from the "bridles" 
####################################################################################

SUBDIRS=motor laser camera common main

####################################################################################
# Name of the final binary
####################################################################################

TARGET=avoidall

####################################################################################
# Describe the directories you want to include from the "bridles" 
####################################################################################

# BLOB for all object files
OBJS=$(wildcard ../obj/*.o)

COPY_ETC_CMD=../etc/copy.sh
COPY_ETC_SRC=../etc
COPY_ETC_DST=../bin

# This setup requires you to call "make". 
# A call to "make all" will screw everything up!

.PHONY: $(TARGET) all check-env copyobj $(SUBDIRS)

$(TARGET): check-env all copyobj
	$(CXX) $(CXXDEFINE) -o ../bin/$@ $(OBJS) $(CXXFLAGS) $(LDFLAGS)
	$(STRIP) ../bin/$@
	$(CSIZE) ../bin/$@

check-env:
ifndef EQUID_PATH
    $(error EQUID_PATH is undefined)
endif

upload: all obj
	$(STRIP) ../bin/$(TARGET)
	#cat ../bin/robotServer|netcat -l -p 7878 

createdirs:
	mkdir -p ../obj
	mkdir -p ../bin

subdirs: $(SUBDIRS)
	@echo "Make all subdirectories" 

$(SUBDIRS):
	@echo "Make subdirectory" $@ 
	$(MAKE) -C $@

all: clean createdirs subdirs

copyobj: all
	@echo "Copy objects from subdirs to \"obj\" directory"
	@for i in $(SUBDIRS) ;\
	do \
	echo "Copying all from $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o ../obj; \
	done

clean:
	@for i in $(SUBDIRS) ;\
	do \
	echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i clean; \
	done
	@echo "cleaning all objs"
	@rm -f ../obj/*.o
	@rm -f ../bin/$(TARGET)
