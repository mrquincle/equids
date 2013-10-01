#!/bin/sh
#OPTIONS=-q # actually feedback on killing is nice
#OPTIONS=-v # does not exist on robot
killall $OPTIONS docking
killall $OPTIONS movetoposition
killall $OPTIONS mapping 
killall $OPTIONS ubiposition
killall $OPTIONS actionselection
killall $OPTIONS cameradetection 
killall $OPTIONS motorcalibration 
killall $OPTIONS zigbeemsg
killall $OPTIONS remotecontrol
killall $OPTIONS organismcontrol
killall $OPTIONS avoidir
killall $OPTIONS laserscan
killall $OPTIONS wenguo
