ADDITIONAL_FLAGS=-DBOOST_NO_INTRINSIC_WCHAR_T -DBLACKFIN

CXXFLAGS+=$(ADDITIONAL_FLAGS)
LXXLIBS+=-L/opt/uClinux/bfin-linux-uclibc/lib/gcc/bfin-linux-uclibc/4.3.5 -L/opt/uClinux/bfin-linux-uclibc/bfin-linux-uclibc/
LDFLAGS+=$(ADDITIONAL_FLAGS) -s -L$(WAPI_LIB_PATH) -lWAPI

