# finals_turret
A project for the hardware and software of a simple turret replica from the greatest game show on earth, The Finals.

## Table of Contents
- [Requirements] (#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Future_Additions](#future_additions)
- [Configuration](#configuration)

## Requirements
- Software:
    - Freecad (for 3D part design)
    - KICAD (PCB design)
    - ST Cube IDE (ST account required for download)
- Hardware:
    - BOM to come
    - Access to a 3D printer

## Installation
No code to show yet.

Current Turret design is not great and I should have designed it in pieces to be assembled. Too much support material is needed and its hard to remove without breaking the parts.

Here is the original model from the finals that the project is based on.
![Game Model Its Designed After](/Pictures/ActualGameModel.png)

And here is my design made in Freecad.
![Turret Assembly](/Pictures/Assembly.png)

Its not yet optimised for 3D printing but I believe its possible to make small changes to reduce the ammount of support without affecting the looks.

FInally here is the brains of the project.
![PCB](/Pictures/AssembledPCB.jpg)

I assembled this PCB by hand using a soldering iron but I would recommend using assembly services from a manufacturor like JLCPCB.

## Usage


## Features
- Audio playback using a dfplayer mini
- Programmable using SWD or USB
- USB serial communications for debugging or remote control

## Future_Additions
- Raspberry pi python code for tracking people with a camera
- New and improved PCB with its own on board audio amplifier + SD card slot or some other form of audio storage

## Configuration