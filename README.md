# Drawing Robot - DAKOTA BLACK

A wheeled robot that interprets G-code to draw images, may or may not include methods of tracking to maintain alignment. 

## Summary

This project will:
- create a vector tracing of an image
- convert the vector drawing to gcode.
- store gcode on a sd card 
- set the starting point as the origin
- read gcode line by line
- execute x and y comands to postition the robot.
- execute z up and down comands to raise and lower a pen 
- keep track of wheel movements
- keep track of pen up and pen down 


## Component Parts

The base of the robot will be an adafruit kit, with:
- geared dc motors for the wheels
- a servo to raise and lower the pen
- a motorshield to run the dc motors
- rotory encoders or light/color sensors to keep track of the wheels
- an sd card and sdcard shield to store the Gcode


Include what types of inputs/outputs/data it will use, and a block diagram showing how all those pieces are connected.

## Challenges

keeping track of the wheels and moving the robot accurately and tracking the movement of the robot pose the greatest chanllenge.

## Timeline

- Week 1: Write proposal, basic prototype
- Week 2: pen up & down, wheel tracking, postion tracking
- Week 3: gcode from sd card 
- Week 4: fine tune!
- Week 5: Present!

## References and link
 
  http://reprap.org/wiki/Arduino_GCode_Interpreter
  
  http://www.instructables.com/id/Arduino-Drawing-Robot/
  
  https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial?view=all
  
 ![pinout](pinout.png)
