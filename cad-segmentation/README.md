
# CAD Segmentation

## What is this

This folder contains the code and scripts for the **CAD segmentation stage** of the robot-based laser structuring master thesis project.  

In this stage, a given CAD model is segmented into smaller surface patches / segments. These segments will later be used to compute robot paths and laser toolpaths for laser structuring with a robot.


## Folder Overview

/scripts

1. main_with_angles.py 
- This is the first of the 2 main scripts that performs cad segmentation.
- This uses point normals.

2. main_with_angles2.py **(recommended)**
- This is the second of the 2 main scripts that performs cad segmentation.
- It is same as first one but uses cell normals instead. 

Both these scripts segment a CAD mesh into clusters using K-Means, computes each segment’s centroid and normal projected onto the surface, optimize the visit order using TSP, and export the ordered centroid-normal pairs as a robot toolpath while visualizing the segmentation results.

3. main.py
- This script does cad segmentation but only stores centroids and not their normals into a file. This was the initial script to test segmentation that was later upgraded into the the scripts with angles considered.

4. rapidcode_jointtarget.py
- This script creates a RAPIDCODE module consisting of **jointtargets** for ABB RobotStudio from the saved segmented data (centroids+normals). It was created to validate the CAD Segmentation phase.

5. rapidcode_robtarget.py **(recommended)**
- This script creates a RAPIDCODE module consisting of **robtargets** for ABB RobotStudio from the saved segmented data (centroids+normals). It was created to validate the CAD Segmentation phase.

6. test.py
- This was created to test loading and slicing of different models in Pyvista.

