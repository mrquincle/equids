# Equids

The jockeys have to ride something... :-) This can be seen as a modular application software approach. The software in this directory uses the old-fashioned irobot middleware, but it can use the new HDMR+ middleware through the wrapirobot glue code.

This directory is divided into several subdirectories:

- bridles: the code to drive the wheels, get the camera and laser data, etc.
- jockeys: the controllers that for example detect a power outlet, prevent collisions, etc.
- managers: server-side code that for example uses X

## Bridles

Can be seen as another wrapper around the middleware files. However, it provides useful functionality, rather than only basic ones. One example, the CRawImage file can write and read images to file.

## Jockeys

The controllers for the horses (the robots). To enable reuse, these controllers use symbolic links to the bridles.

## Managers

The server side utilities that for example show the camera images coming from the robots. Preferably everything is written such that it can run on a robot, but this is not always feasible.

## Compilation 

The only thing that is required for compilation is to set one environmental variable:

    cd "the path where you checked out this repository"
    export EQUID_PATH=`pwd`

## Copyright

Something like BSD / LGPL, contact us to make sure.
