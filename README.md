# OHTI-HT-osc-wifi

 What is OHTI-HT  (Open Head Tracker Implementation), it is a result of the effort to create a low cost HT solution. 

ToDo
--------
Add LED indication of calibration stage! (Steady light , then fast flashes?)!
Store and reload BNO055 Calibration over PowerOff in program flash!
The OSC port for UDP broadcast is fixed at 9000, make it configurable?
Add OSC support over websocket to control Omnitone webplayer.
Do not send any osc message if the headtracker is stationary?

Design choices: 
-------------------------
Originaly BNO055 IMU was used to minimize the programming effort and cpu load.
Several versions of headtracker message protocols have been tried.
OSC WiFi UDP is chosen for this version, only drawback is the current consumption of the WiFi communication.

Currently the OSC message is configured to use the message syntax of the IEM SpatialRotator VST.

The support of OSC message for IEM Spatial Rotator is a good combination with the IEM binaural decoder with SOFA support on Reaper DAW.

OSC with quaternions carry the directional info, this to simplify the directional reset calculations and avoid gimbal lock.

The low cost HW solution is Wemos D1 mini ( esp8266 chinese module) and chinese BNO055 with external crystal.
Drawback, 4 connections needs to be soldered between the 2 modules, I have used Aliexpress as hw supplier.

For a no solder solutions the quiic system and bno080 and esp8266 from sparkfun can be used with software changes(I guess).


Setup
---------
At initial power up or if the previously configured network is unreachable, enter the credentials to use a available WiFi network:

Connect a computer to the wifi network OHTI AP.
Access the webpage .......
Enter the credentials for the local WiFi network to use.

Click the save ... button, after the storage of the credentials the OHTI HT will reset and connect to the configured wifi network.

If the configured network is not available at power on the OHTI AP will be started.

Power the OHTI-HT by a usb power bank, is a suggestion if you do not power it from your computer. The power bank can be put in a pocket or at your belt.

Usage
----------
After power on:
During HW initilization the blue LED will be lit, when it is turned off KEEP the OHTI-HT stable on flat surface.

Calibration procedure - wait 30 seconds after power on with sensor still in horizontal position,
Next step, When LED turns off, MOVE sensor in a FIGURE of EIGHT in a vertical direction with usb cable connector as rotational center for about 20 seconds until the LED lighthes up.

Look at the image in docs  directory, i have problem with the html addition
<p align="center">
  <img src="doc/ohtiht.jpg" width="350" title="Connections photo"></p>
  
<p align = "center">
<img src https://github.com/bossesand/OHTI-HT-osc-wifi/tree/master/docs/ohtiht.jpg>
</p>


