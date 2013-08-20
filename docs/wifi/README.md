
# Wifi

Many people wanted to know how to enable wifi on the ActiveWheel. So, here is how you can configure it to wirelessly connect to a network starting from factory settings:

* start robot, login (root, no password)
* change the /flash/ethsetter.sh file to use IP 192.168.2.100
* reboot now # and login again
* telnet 192.168.2.1 # on the robot to the Quatech, this IP address is the factory default
* auth dpac dpac # gives you a console for custom commands, type help e.g.
* wl-scan # scans the available SSIDs
* wl-ssid "some network name" # to make it automatically connect to that SSID
* commit # will store your changes to flash on the Quatech device
* restart # will restart the device
* wl-ip # gives you the new IP address of your device over which you can access it from remote computers, equivalently you might find out by going to the router and looking through its table of connected clients, or use an arp-scan to scan the local network from another attached computer

So, now you configured the Quatech. This might very well not be necessary later on. You can use the webpage it serves on port 80 to configure things differently later on. Most importantly, you know now how to reach it over ethernet.

Set the IP address as defined in Replicator/Symbrion. So, for the ActiveWheel robot nr.4 becomes with +50, and netmask 192.168.52.X, the IP address: 192.168.52.54. 

* change the /flash/ethsetter.sh to 192.168.52.X
* reboot, and check ifconfig

Now, we can check if we can see the ActiveWheel popping up in our network:

* sudo arp-scan -l -I wlan0 # on the remote computer, will give you something like this:

````
Interface: wlan0, datalink type: EN10MB (Ethernet)
Starting arp-scan 1.8.1 with 256 hosts (http://www.nta-monitor.com/tools/arp-scan/)
192.168.1.1      00:22:6b:7c:56:de    Cisco-Linksys, LLC
192.168.1.100    00:0b:6b:7e:cc:4b    Wistron Neweb Corp.
````

* ping 192.168.1.100 # I added this, because it will NOT work. I know, don't email me about it :-), use arping instead

You can access the Quatech now by the DHCP address via the wireless network:

* firefox http://192.168.1.X # press F5 to make sure it reloads and disable javascript blockers!
* fill in "dpac" and "dpac" for username and password
* change stuff # in Replicator/Symbrion in the ARENAnet the robots are configure with the same IP address using a different mask, so 192.168.52.X internal to the robot (on the ethernet side of the Quatech) becomes 192.168.1.X on the external side of the robot (the wireless side of the Quatech)
* restart and reboot everything

Route on your laptop, suppose you have ActiveWheel nr.4:

* sudo route add -net 192.168.52.0 netmask 255.255.255.0 gw 192.168.1.54 # gateway is the WLAN side of the Quatech

* telnet 192.168.52.54 # success! access to the robot from your laptop

## How to route from another robot?

You will need to add a filtering rule, because I didn't figure out how to disable the firewall yet:

* go in the browser to http://192.168.1.X (dpac, dpac), then to the configuration tab, then to "Wireless Routing Settings"
* add a rule: FORWARD, and DEST IP: 192.168.52.217 for TCP port 10002 # in case you want to stream images from port 10002 to a server from Scout 17 over the ActiveWheel

* telnet 192.168.52.217 # should work now (the telnet port is always going through, without the forward rule)

Just test it the other way around, on the second robot (the Scout):

* route add -net 192.168.1.0 netmask 255.255.255.0 gw 192.168.52.1 dev eth0

So, the Quatech chip in the ActiveWheel (so there is only one in this organism) can be reached at 192.168.52.1. Now you can test it from the Scout to your laptop (which is, say, at 192.168.1.2):

* telnet 192.168.1.2

This should always get through because it is outbound.

## How to send files?

* cat FILE |  netcat -l -p 10000 # on remote laptop side to send a file to the robot
* nc 192.168.1.2 10000 >FILE # on the robot side, to receive the file, the 192.168.1.2 address is your laptop

Or use rpc, it is now enabled on the robot.

