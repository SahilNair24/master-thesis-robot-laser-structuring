
# Laser Path Planning

## What is this

This folder contains the scripts for the **Laser Path Planning stage** of the robot-based laser structuring master thesis project.

In this stage, a 2D pattern is created on a surface using FreeCAD and a gcode file is generated containing an ordered path that the laser scanner must follow to create the desired laser structuring pattern.


## Folder Overview

/freecad

gcode_to_mpf.py
- This script converts the gcode file exported from FreeCAD into a mpf file in the native dialect used at Fraunhofer IPT for laser structuring operations
gcode -> mpf conversion

