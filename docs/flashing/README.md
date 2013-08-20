# Flashing Replicator robots

Let's hope I can put something down here what is gonna help everybody to work with the robots as much as Launchpad did for the Robot3D simulator. Regarding all upcoming sections in this email, there are experts, who know much more than me, but I feel like I am one of the few people who might give you the full user experience. This is about Popeska, the ActiveWheel robot. We had a great time together. :-)

This email is especially meant for people that will get a robot over the mail and need to start working with it. I have divided this email in three sections. First the repositories that you might need. Second, a section on MSP. Third, a section on the Blackfin. If you start with the robot you can skip the entire MSP section and go straight to the section on the Blackfin. There you will only need to learn the IRobot API, and how to get files back/forth to the Blackfin. I hope this email might also relieve Florian and Christopher a bit from their duties as "jack of all trades".

## Repository

* MSP: https://ipvs.informatik.uni-stuttgart.de/software/repos/software/msp
* irobot: https://ipvs.informatik.uni-stuttgart.de/software/repos/software/blackfin/irobot
* uclinux binary images: http://ipvs.informatik.uni-stuttgart.de/software/repos/software/blackfin/uclinux

There are currently no people who feel responsible for maintaining these repositories. If there will be, the most important thing I recommend is a readme file in each project, which is the most up to date file or branch to use (and maybe a list of bugs yet to solve). 
 
## MSPs

The robot contains four boards with MSPs besides the main board with the Blackfin (BF). Each of the boards communicates with the BF via SPI. The devices are listed in SPIStream.h in the irobot library:

    {"SPI A HINGE",     "/dev/spidev0.2", 10, SPITYPE_MSP},
    {"SPI B FRONT",     "/dev/spidev0.3", 17, SPITYPE_MSP},
    {"SPI C SIDE",      "/dev/spidev0.4",  8, SPITYPE_MSP},
    {"SPI D POWER",     "/dev/spidev0.5",  9, SPITYPE_MSP},

The individual boards use a general pin to indicate that they have something to be read for the BF. For now, these pins seem to be floating which causes the speed of transmission to go down. Flow for incoming stuff: ISR_DMA -> ReadRXDMA -> TriggerEvent( EVENT_SPIMSG ) -> SPIMessageHandler -> processCommand with immediate response: ISR_DMA -> WriteTXDMA & ClearGPIO. And the other way for outgoing (microphone) data: ADC12_ISR -> MICSendStatus -> SPISend -> SPISendMessage -> SPIWriteBlocking -> SPIWrite -> BQPushBytes & WriteTXDMA & ClearGPIO. It is important to notice that SPISendMessage drops messages if the buffer is full. Take care if you use LEDs to indicate status of something send. LEDs are also SPI commands, so they interfer! As a user I can say that the MSPs seem to work quite reliably in the sense that they do not seem to be segfaulting when running for extended times. However, in the high througput case of microphone data on average 1 of every 2 packages get lost. So, take this in consideration if you rely on (sensor) data from the MSPs. 

## Program an MSP

sudo aptitude install gcc-msp430 gdb-msp430 mspdebug

* make ACTIVE_WHEEL
* sudo mspdebug uif -j -d /dev/ttyUSB0
* prog aw_side_board.hex # do not use load instead of prog, that doesn't work
* prog scout_left.elf
* reset

## Program the Blackfin

Florian and Christopher (and others) have created this nice irobot library that can be used for programs for partners. For now if you link your program to irobot you will need to have superuser rights. You cannot run it as the default user "alice". Be prepared that you screw up badly sometimes and need to reflash the kernel (see below). To flash your own programs to the blackfin you can use minicom. 

* sudo minicom -c on -s
* Filenames and paths: choose upload/download directory
* Serial port setup: /dev/ttyUSB0 (or 1) and 115200 8N1 bps/par/bits
* Modem and dialing: delete A through H
* Safe setup as dfl
* Exit

And now start minicom jus as user: minicom -c on -s. You can flash something to the robot by Ctrl+A S and selection by spaces. You can download something from the robot by typing sz file* where wildcards work.

From a user perspective, the irobot library is quite easy to interface with. The IRobot.h file is the only one the user needs to know if everything works fine. As with the MSP, when the SPI bus goes faster than 5kHz, the Memmo implementation (which functions as a buffer towards the user) starts loosing messages.

To program the Blackfin you do not need to have any additional hardware except for the connector itself. Ask Florian for the pin layout. The toolchain for the Blackfin can be obtained from the repository from Stuttgart, but again I will show the easy way if you are running Ubuntu 11.10. Go to http://docs.blackfin.uclinux.org/doku.php?id=toolchain:installing where you will find a section "installing packages on a debian-based distro" that ends with sudo apt-get install blackfin-toolchain-uclinux blackfin-toolchain-linux-uclibc You will download directly from a repository from analog.com. And you would be able to start to create your own programs after typing 15 lines in the shell.

For the people who do not have a Replicator robot available and still want to test code on a robot running a Blackfin I recommend the Surveyor SRV-1. Regarding the projects hardware resource allocations this might be your only shot to have working code.

The current kernel is probably the one at http://ipvs.informatik.uni-stuttgart.de/software/repos/software/blackfin/uclinux/2010R1-RC1/ and not the one in the kernel directory. However, email Rene and Samiksha to get the newest kernel. On the moment I am using the uImage.ext2.nfs on my harddrive (which comes from the repository and is placed there by Samiskha). The following procedure shows how to use kermit with minicom. You can also use ymodem instead of kermit. Or netcat instead of minicom, etc. On Ubuntu installation is again easy:

* sudo aptitude install minicom ckermit
* vim ~/.kermrc
* set line /dev/ttyUSB0
* set speed 115200
* set carrier-watch off
* set handshake none
* set flow-control none
* robust
* set file type bin
* set file name lit
* set rec pack 1000
* set send pack 1000
* set window 5

Make sure of course that it is indeed device ttyUSB0.

For the rest to use minicom and kermit from uboot, use http://lists.denx.de/pipermail/u-boot/2003-June/001527.html 

Logs from minicom:

    loadb
    ## Ready for binary (kermit) download to 0x01000000 at 115200 bps...                                                             
    ## Total Size      = 0x002df035 = 3010613 Bytes                                                                                  
    ## Start Addr      = 0x01000000

Store this number 0x002df035 somewhere, it is $(filesize) in the Blackfin User guide.

You can check if the downloaded images is correct.

    bfin> imi 1000000
    ## Checking Image at 01000000 ...
       Legacy image found
       Image Name:   Linux Kernel and ext2
       Image Type:   Blackfin Linux Kernel Image (gzip compressed)
       Data Size:    3010549 Bytes =  2.9 MB
       Load Address: 00100000
       Entry Point:  00318aa8
       Verifying Checksum ... OK

Now copy it actually to the right flash section:

* protect off 0x20040000 +0x002df035; erase 0x20040000 +0x002df035; cp.b 0x1000000 0x20040000 0x002df035; protect on all
* cmp.b 0x1000000 0x20040000 0x002df035
* bootm 0x20040000

The 0x002df035 is the filesize of before. Different from most manuals we only remove the protection from the place where we want to write. We only erase that part. And we copy from 0x1000000 which is where loadb by default puts it on the BF561. The compare option is of course not really necessary but checks if the copying process was correct. And then we boot the image, ready!


