# GRBL-VESC
Modified grbl 1.1g (Mega2560 only) to communicate with a vesc over uart (used as spindle).

grbl-mega: https://github.com/gnea/grbl-Mega

VescUart: https://github.com/SolidGeek/VescUart

VESC:
- https://vesc-project.com/
- https://github.com/vedderb/bldc
- https://github.com/vedderb/bldc-hardware
- https://github.com/vedderb/bldc-tool

Grbl communicates with the vesc over the serial1 port of the arduino mega (pins 18, 19) at baud 115200. 
Before connecting remember that the vesc runs at 3.3v while the arduino runs at 5. You will need a logic level shifter.
Don't forget to configure the vesc to accept uart commands at the right baudrate.
