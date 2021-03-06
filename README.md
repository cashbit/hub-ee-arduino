hub-ee-arduino
==============

Sketch to drive a robot with [HUBee Wheels](http://www.creative-robotics.com/?q=About-HUBee-Wheels) and Arduino.


## Install

You need to download the [library](http://www.creative-robotics.com/sites/default/files/software/bmd-s/HUBeeBMDWheel_arduinoLib.zip) from [this page](http://www.creative-robotics.com/?q=hubee-resources): and install it in the Arduino IDE.

## Driving

To drive send these commands on the Ardunino serial port, followed by CR to confirm:
```
  eme:0 --> emergency stop

  dir:1 --> set direction, 1 forward, 0 backward
  
  dis:100 --> set distance, motor will go in the setup direction for 100 steps
  
  vel:0 --> set speed, 0 to stop, 255 for max speed
  
  rot:90 --> rotate clockwise 90 steps, with -90 rotate counterclockwise 90 steps
```

The robot sends back on the serial the status with a json string like this: 

```
{"sx":{"speed":100,"counter":23,"direction":1},"dx":{"speed":100,"counter":23,"direction":1},"distanceleft":103.4}
```

When distanceleft equals 0, the robot has runned for the requested steps with dis command.
 
You can concatenate many commands in one line, examples:

- make 100 steps forward, max speed

```
  dir:1,dis:100,vel:255
```
- rotate by 90 steps on the right at speed 100

```  
  rot:90;vel:100
```
  
Note: every time the robot is approaching the distanceleft 0 value, it sets automatically the speed to 50 to slow down (only if the speed is more than 50).


## Hardware

The hardware needed to start:

- Two [HUB-ee wheels](http://www.creative-robotics.com/?q=catalog/1)
- Two [HUB-ee Open Collector break out board](http://www.creative-robotics.com/?q=node/483)
- Two [HUB-e cables](http://www.creative-robotics.com/?q=node/70)
- One [Arduino](http://arduino.cc) (tested on 2009, UNO is ok)
- USB Cable to connect Ardunino
- Some wires to connect the break out board with Arduino (and a few time to solder them), please refer to the code for the wiring
