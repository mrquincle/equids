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

####################################################################################
# List the directories you want to include from the "bridles" 
####################################################################################

SUBDIRS+=common
SUBDIRS+=motor
SUBDIRS+=eth
SUBDIRS+=move_to
SUBDIRS+=main

OBJS=$(wildcard ../obj/*.o)

# Expects now an IROBOTPATH that is set globally or in the /etc/robot/overwrite.mk directory
#LXXLIBS+=-DBOOST_NO_INTRINSIC_WCHAR_T -DBLACKFIN -s -L../../wapi/wapi -lWAPI
COPY_ETC_CMD=../etc/copy.sh
COPY_ETC_SRC=../etc
COPY_ETC_DST=../bin

ifeq ($(MULTI_CONTROLLER),true)
LXXMULTI=-lrt
endif

movetoposition: all obj
	$(CXX) $(CXXDEFINE) -o ../bin/$@ $(OBJS) $(CXXFLAGS) $(LDFLAGS) 
	$(STRIP) ../bin/$@
	$(CSIZE) ../bin/$@

#	$(CXX) $(CXXDEFINE) -o ../bin/$@ $(OBJS) $(CXXFLAGS) $(LXXLIBS) -lpthread -Ldrive 
#	/opt/uClinux12/bfin-linux-uclibc/bin/bfin-linux-uclibc-strip ../bin/$@
#	cp ../bin/$@ ../../../poslat/$@
#	rcp ../bin/$@ root@192.168.2.15:/flash/
upload: all obj
	cat ../bin/robotServer|netcat -l -p  10000 


all:
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all; \
	done

obj: all
	echo "Copy objs"
	@for i in $(SUBDIRS) ;\
	do \
	echo "coping all in $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o ../obj; \
	done

etc: ../bin/.etc

	#cp -r ../etc/$$i ../bin; \
	#
../bin/.etc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

cleanetc:
	rm -f ../bin/.etc	

	#cp -r ../etc/$$i ../bin; 
forceetc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

clean:
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i clean; \
	done
	echo "cleaning all objs"
	rm -f ../obj/*.o
	rm -f ../bin/*
