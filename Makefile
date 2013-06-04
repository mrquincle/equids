include Mk/default.mk

####################################################################################
# Explanation
####################################################################################

# Of course you can have a for-loop in a Makefile, however, such a loop is a SHELL 
# loop. This means, that an error will not be automatically propagate back to this
# Makefile and it will continue building the other folders. It can be hard to see
# errors in that way. Manually catching the error does not respect "make -k". 
# Moreover, with a for-loop the make cannot be executed in parallel.

JOCKEYS=jockeys/lasertest jockeys/laserscan

jockeys: $(JOCKEYS)

$(JOCKEYS):
	$(MAKE) -C $@

all: check-env jockeys

check-env:
ifndef EQUID_PATH
  export EQUID_PATH:=$(CURDIR)
endif

# List all the phony targets
.PHONY: jockeys $(JOCKEYS) all

