include Mk/local.Mk
SUBDIRS+=common
SUBDIRS+=control
SUBDIRS+=gui
SUBDIRS+=main
SUBDIRS+=main
LXXLIBS+=-lSDL -lpthread

OBJS=$(wildcard ../obj/*.o)

COPY_ETC_CMD=../etc/copy.sh
COPY_ETC_SRC=../etc
COPY_ETC_DST=../bin

visualizer: all obj
	$(CXX) $(CXXDEFINE) -o ../bin/$@  $(OBJS)  $(CXXFLAGS) $(LXXLIBS) 

cameraminoru:
	rm -rf camera
	ln -s camera.minoru camera


cameraweb: 
	rm -rf camera
	ln -s camera.web camera

cameradummy: 
	rm -rf camera
	ln -s camera.dummy camera

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
	echo "cleaning binaries"
	rm -f ../bin/*
