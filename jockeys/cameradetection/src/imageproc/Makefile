# It is possible to compile a "bridle", but it only makes sense if a "jockey" uses it to control a robot.
# Compile it separately for debugging purposes.

# IMPORTANT!! Set EQUID_PATH as environmental variable, if you compile this separately, for example:
#   export EQUID_PATH=$HOME/myworkspace/stuttgart/controller/equids

# Load default Makefile for a bridle in the jockey framework 
-include $(EQUID_PATH)/Mk/default.mk
# Override default Makefile options with a local Makefile
-include $(EQUID_PATH)/Mk/local.mk

# By default grab only all .cpp files to compile
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))
OBJSC=$(patsubst %.c,%.o,$(wildcard *.c))
OBJS+=$(OBJSC)

# The directories that this "bridle" depends on
CXXINCLUDE+=-I./  -I../common -I../map -I../camera -I../../../../libs/gsl

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
