# Collision avoidance using infrared

This controller performs a basic form of collision avoidance using infrared. It uses a lot of code from Wenguo Liu (from Bristol university) and is meant for the Replicator / Symbrion robots in the same EU projects. 

It contains a calibration procedure. What this basically does is turning the robot on the spot (so it needs access to the motors) and calculating a smoothing average over time. This value is used as an offset by all subsequent readings from the infared sensors.

There are several modes from reading from the infrared sensors. Ambient light is fed into the sensors at all time. However, for collision avoidance the so-called "reflective" values are used. The LEDs are actually turned on, and the reflection on a surface is measure and used to indicate a distance.


