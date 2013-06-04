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

SUBDIRS+=laser
SUBDIRS+=main

####################################################################################
# Name of the final binary
####################################################################################

TARGET=lasertest

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

.PHONY: $(TARGET)

$(TARGET): check-env all obj
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

makedirs:
	mkdir -p ../obj
	mkdir -p ../bin

.PHONY: all
all: clean makedirs 
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all; \
	done

obj: all
	@echo "Copy objects from subdirs to \"obj\" directory"
	@for i in $(SUBDIRS) ;\
	do \
	echo "Copying all from $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o ../obj; \
	done

etc: ../bin/.etc
	#cp -r ../etc/$$i ../bin; \
	#

../bin/.etc:
	@echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

cleanetc:
	rm -f ../bin/.etc	

	#cp -r ../etc/$$i ../bin; 
forceetc:
	@echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

clean:
	@for i in $(SUBDIRS) ;\
	do \
	echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i clean; \
	done
	echo "cleaning all objs"
	rm -f ../obj/*.o
	rm -f ../bin/$(TARGET)


