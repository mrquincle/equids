#!/bin/make -f

# make blobfinder of Prague

all:
	mkdir -p bin obj
	cd src && make

clean:
	@echo "Clean content of bin directory yourself"
	rmdir bin
	rm obj/*
	rmdir obj
	cd src && make clean
