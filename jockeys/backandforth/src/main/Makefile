# It is possible to compile a "bridle", but it only makes sense if a "jockey" uses it to control a robot.
# Compile it separately for debugging purposes.

# Load default Makefile for a bridle in the jockey framework 
include $(EQUID_PATH)/Mk/default.mk
# Override default Makefile options with a local Makefile
-include $(EQUID_PATH)/Mk/local.mk

# By default grab only all .cpp files to compile
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

# The directories that this "bridle" depends on
CXXINCLUDE+=-I./ -I../eth -I../common

all: $(OBJS) 

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS) *.moc $(UI_HEAD) $(UI_CPP)
