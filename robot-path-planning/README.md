
# Robot Path Planning

## What is this

This folder contains the scripts for the **Robot Path Planning stage** of the robot-based laser structuring master thesis project.  

In this stage, a 2D pattern is created on a surface using FreeCAD and a gcode file is generated containing an ordered path that the laser scanner must follow to create the desired laser structuring pattern.


## Folder Overview

/abb_irb6660_moveit_config\
This folder contains a MoveIt configuration package specifically for the ABB IRB 6660 robot. It includes the robot’s kinematic and semantic descriptions (URDF/SRDF), motion-planning parameters, controller and launch files needed to run MoveIt for planning and executing robot trajectories within your path-planning pipeline using ROS/MoveIt.


/abb_irb6660_support
Provides the ABB IRB 6660 robot model and support files for ROS/MoveIt motion planning


/robotpath
This is the main ROS 2 package that is responsible for robot motion planning using Moveit2.
The package contains ROS 2 nodes and launch files.

Launch files:
1. demo_urdf.launch.py 
- Launches the robot model of ABB IRB 6660 along with the URDF of the surface to be segmented so both can be visualised together in one simulation environment

2. motion_plan.launch.py
- This launch file is responsible for starting the motion planning for the robot using Moveit2. It reads the poses from the csv file obtained from CAD Segmentation stage and plans and executes motion for the robot to move through the defined poses.

Nodes:
1. fk_service.py
- This node was created to validate the trajectories created using Moveit2 in the robot path planning stage. This node is used to convert robot trajectory obtained in the form of joint angles to cartesian poses by using the forward kinematics service of Moveit2. 
