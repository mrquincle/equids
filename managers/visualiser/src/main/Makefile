OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

include ../Mk/local.Mk
include ../../../../Mk/paths.mk

# overwrite sorry
WAPI_PATH=/home/gestom/svn/replicator/blackfin/controller/equids/libs/wapi64bit/include
WAPI_LIB_PATH=/home/gestom/svn/replicator/blackfin/controller/equids/libs/wapi64bit/Laptop

CXXINCLUDE+=-I./ -I../common -I../camera -I../gui -I../control -I../zigbee -I$(WAPI_PATH) 
CXXINCLUDE+=-I/usr/local/include

# add zigbee and wapi
LXXLIBS+=-DGLIBCXX__PTHREADS -DBOOST_NO_INTRINSIC_WCHAR_T -s -L$(WAPI_LIB_PATH) -L/usr/lib -lWAPI -lpthread -lboost_thread-mt 

all: $(OBJS) 

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS) *.moc $(UI_HEAD) $(UI_CPP)
