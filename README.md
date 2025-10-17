# finals_turret
A project for the hardware and software of a simple turret replica from the greatest game show on earth, The Finals.

## Table of Contents
- [Requirements](#requirements)
- [Rationale](#rationale)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Future_Additions](#future_additions)
- [Configuration](#configuration)
- [Known_Issues](#known_issues)

## Requirements
- Software:
    - Freecad (for 3D part design)
    - KICAD (PCB design)
    - ST Cube IDE (ST account required for download)
- Hardware:
    - BOM to come
    - Access to a 3D printer

## Rationale
- STM32f411:
    - This is a pretty overkill MCU for a project such as this. It even features I2S protocol which would make it possible for me to replace the dfplayer mini module with my own custom on board audio amplifier. The whole reason to go with this 100MHz MCU is because I want to use it for a drone project so I am testing if it works as expected first with this project. It also makes things faster for the drone project if I can copy and paste the schematic.

## Installation
No code to show yet.

Current Turret design is not great and I should have designed it in pieces to be assembled. Too much support material is needed and its hard to remove without breaking the parts.

Here is the original model from the finals that the project is based on.
![Game Model Its Designed After](/Pictures/ActualGameModel.png)

And here is my design made in Freecad.
![Turret Assembly](/Pictures/Assembly.png)

Its not yet optimized for 3D printing but I believe its possible to make small changes to reduce the amount of support without affecting the looks.

FInally here is the brains of the project.
![PCB](/Pictures/AssembledPCB.jpg)

I assembled this PCB by hand using a soldering iron but I would recommend using assembly services from a manufacturer like JLCPCB. Dont forget to use a generous amount of flux if soldering the stm32 by hand.

This is how the PCB should fit into the turret. The whole thing uses M2.5 5mm screws for the speaker mounting and PCB mounting.
![PCB in enclosure](/Pictures/PCB_IN_Enclosure.jpg)

The servos mounting positions have mounting holes that should work with the self tapping mounting screws that are usually provided with 9g servos. The servo should be oriented so that the output rotation is in the same axis as the center of rotation for the pan and tilt respectively.

## Usage


## Features
- Audio playback using a dfplayer mini
- Programmable using SWD or USB
- USB serial communications for debugging or remote control
- USB C (2.0)
- Power input 5-12V (can use a normal 12V router power cable)
- Reverse polarity protection
- Buck converter from input stepped down to 5V for efficiency.
- Two PWM output ports to control servo motors.
- Lots of extra exposed MCU pins for rework or future quick additions.
- Potentiometer for volume control

## Future_Additions
- Raspberry pi python code for tracking people with a camera
- New and improved PCB with its own on board audio amplifier + SD card slot or some other form of audio storage

## Configuration

## Known Issues
- The standoff for the dfplayer mini is the wrong way around.
- The port for the USB and barrel jack connector should be at the back and not the side.
- The 3.3V LDO large pin is in fact connected to Vout and not GND. I need to fix the footprint in KiCAD.