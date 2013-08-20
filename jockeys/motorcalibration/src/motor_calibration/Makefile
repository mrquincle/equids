include ../Mk/localBF.mk
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))


CXXINCLUDE+=-I./ -I../common -I../motor -I../eth -I../../../../libs/gsl
CXXINCLUDE+=-I/home/robot/include
CXXINCLUDE+=-I$(IROBOTPATH)
CXXFLAGS+=-std=c++0x
all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
