CXX=/opt/uClinux/bfin-linux-uclibc/bin/bfin-linux-uclibc-g++
LXXLIBS+=-L/opt/uClinux/bfin-linux-uclibc/lib/gcc/bfin-linux-uclibc/4.3.5
#CXXFLAGS+=-O2 -ggdb -mcpu=bf561 
CXXFLAGS+=-O2 -ggdb -mcpu=bf561 -DMULTI_CONTROLLER -std=c++0x
MULTI_CONTROLLER=true
#BFINCLUDE=/opt/uClinux/bfin-linux-uclibc/bfin-linux-uclibc/runtime/usr/include

IROBOTPATH=/home/robert/Bakalarka/workspace/irobot
