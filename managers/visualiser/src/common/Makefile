OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

CXX= 

include ../Mk/local.Mk
CXXINCLUDE+=-I./ 

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
