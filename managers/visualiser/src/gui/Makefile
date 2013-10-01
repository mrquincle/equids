OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

include ../Mk/local.Mk
include ../../../../Mk/paths.mk

# overwrite sorry
WAPI_PATH=$(HOME)/myworkspace/wapi/wapi
WAPI_LIB_PATH=$(HOME)/myworkspace/wapi/wapi/wapi/WAPI/Laptop

CXXINCLUDE+=-I./  -I../common -I../camera -I../map -I/usr/local/include -I/usr/include -I$(WAPI_PATH)
CXXFLAGS+=-Wall

LXXLIBS+=-DGLIBCXX__PTHREADS -DBOOST_NO_INTRINSIC_WCHAR_T -s -L$(WAPI_LIB_PATH) -L/usr/lib -lWAPI -lpthread -lboost_thread-mt 

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
