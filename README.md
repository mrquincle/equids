# Equids

The jockeys have to ride something... :-) This can be seen as a modular application software approach. The software in this directory uses the old-fashioned IRobot middleware, but it can use the new HDMR+ middleware through the "wrapirobot" glue code.

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

One thing that is required for compilation is to set one environmental variable:

    cd "the path where you checked out this repository"
    export EQUID_PATH=`pwd`
    echo "export EQUID_PATH=$EQUID_PATH" >> ~/.bashrc

If you run everything from the EQUIDS environment, you won't need to set this environmental variable, it will be automatically set in that Makefile.

Now, you can take a look at the file:

    cat $EQUID_PATH/Makefile

If you want to have other jockeys/controllers compiled, create a file called "enable_jockeys.mk" in the $EQUID_PATH and change the array "JOCKEYS" over there. 

### Dependencies

Every jockey has its own dependencies. So, for example a jockey that uses the camera likely needs the libv4l2 library, or the jockey that does the blob detection requires the gsl library. If you are on Ubuntu and want to cross-compile these libraries yourself, the utility [apt-cross](https://github.com/mrquincle/apt-cross) is highly recommended. It uses apt-get to get the source code and has a few configuration options to cross-compile these libraries for the Blackfin microcontroller. Installation is a breeze like:

    apt-cross v4l-utils blackfin
    apt-cross libgsl blackfin

There are no other dependencies known to me.

## Server-side

On the server we require the HDMR+ middleware that has the "wrapirobot" glue layer. This allows you to use the old IRobot middleware calls and requires you to only change a few lines in your controller code. You can find the "wrapirobot" code at https://github.com/mrquincle/wrapirobot.

The HDMR+ middleware is proprietary, created by [Karlsruhe Institute of Technology (KIT)](http://www.kit.edu/english/) and the code can be found at a [Stuttgart University SVN server](http://ipvs.informatik.uni-stuttgart.de/software/repos/software/HDMR-Plus) for Replicator/Symbrion project partners only.

## Copyright

Something like BSD / LGPL, contact us to make sure.
