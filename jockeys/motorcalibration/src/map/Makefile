OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

include ../Mk/localBF.mk
CXXINCLUDE+=-I./ -I../common -I../camera -I../gui -I../imageproc -I../eth -I../../../../libs/gsl
CXXINCLUDE+=-I$(IROBOTPATH)
CXXFLAGS+=-std=c++0x 
all: $(OBJS) 

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
