# It is possible to compile a "bridle", but it only makes sense if a "jockey" uses it to control a robot.
# Compile it separately for debugging purposes.

# Load default Makefile for a bridle in the jockey framework 
-include $(EQUID_PATH)/Mk/default.mk
# Override default Makefile options with a local Makefile
-include $(EQUID_PATH)/Mk/local.mk

# By default grab only all .cpp files to compile
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))
OBJSC=$(patsubst %.c,%.o,$(wildcard *.c))

# The directories that this "bridle" depends on
CXXINCLUDE+=-I./ 

all: check-env $(OBJSC) $(OBJS) 

check-env:
ifndef EQUID_PATH
    $(error EQUID_PATH is undefined)
endif

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CC)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJSC) $(OBJS) *.moc $(UI_HEAD) $(UI_CPP)
