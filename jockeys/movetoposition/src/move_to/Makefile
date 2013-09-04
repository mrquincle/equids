include ../Mk/localBF.mk
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))


CXXINCLUDE+=-I./ -I../common
CXXINCLUDE+=-I/home/robot/include
CXXINCLUDE+=-I../motor
CXXINCLUDE+=-I$(IROBOTPATH)

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
