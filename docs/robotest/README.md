# Robotest

It will be awesome if someone copies this into the binary itself. So, you can just type robotest and 
will see this as a help message. 

````
-------------------------------------------------------------------------------------------------------
               ActiveWheel                           Scout                          KaBot
         robotest Robottesting IRobot     robotest Robottesting IRobot    robotest Robottesting IRobot
 A/front    3         0         front        0          0        front       0          0        front
 B/left     0         1         left         1          1        left        1          1        left
 C/arm      1         2         rear         2          2        rear        2          2        rear
 D/right    2         3         right        3          3        right       3          3        right
-------------------------------------------------------------------------------------------------------

Commands
Description         Command  Parameters
Motor2D               48          2
Unregulated hinge     49          1
Hinge to angle        51          1
RGBLed                52          4
Docking calibration   53          0
MotorEnable           57          3
````

## Docking

Absolutely, first calibrate the docking unit, or else you ruin it:

* ./robotest -c 0 53

only now it's safe to open docking element, do this only after first calibrating the docking unit:

* ./robotest -c 0 50 1

close docking element:

* ./robotest -c 0 50 0

and if nothing works, for debugging, it is possible to just use the motor without the current sensor:

* ./robotest -c 0 52 0 # does nothing
* ./robotest -c 0 52 10 # goes a bit in the opening direction
* ./robotest -c 0 52 -10 # goes a bit in the closing direction

## Leds

* ./robotest -c 0 52 3 3 3 3



