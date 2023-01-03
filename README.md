# Control Strategy for Monitoring Flood and Its Verification in Gazebo

## Summary
- This is the title for my graduation research.
- The objective was to find an inundation in an unkown area using multiple drones.
- A control strategy was developed using an adaptive controller based on Function Approximation Technique Based Immersion and Invariance (FATII), a flood detection algorithm based on image segmentation, and a distribution algorithm based on Centroidal Voronoi Tessellation.
- Gazebo with ROS (Robot Operating System) was used for the simulator environment.
- A ROS package called Hector Quadrotor was used for simulating the drones (17 altogether: 16 for flood detection and 1 for the purpose of overhead view recording).
- The 3D world was created using BLENDER with a GIS (Geographic Information System) plugin and then exported to Gazebo in Collada format.

## How to Install and Run the Simulation
### Prerequisite
- Have ROS Noetic installed. (Install from http://wiki.ros.org/noetic/Installation/Ubuntu)
  - Dependency package: timed_roslaunch.
- Have Hector Quadrotor installed. (Install from https://github.com/RAFALAMAO/hector-quadrotor-noetic)
- Have Python 3 (or above) installed.
  - Required libraries: panda, shapely, opencv, geovoronoi, etc.

### My Environment
- Ubuntu version: 20
- ROS version: Noetic 1.15.11
- Gazebo version: 11.5.1 (This should comes together with the ROS version installed)
- Python version: 3.8.5

### Installing My Project
After making sure you have all the prerequisites installed, you can begin installing my project. This is the way I have tried to install it and it worked. Before that, if you are not familiar with ROS, I recommend doing the ROS beginner tutorial (http://wiki.ros.org/ROS/Tutorials).
- Create a ROS package flood_monitor_16 using the ROS command catkin_create_pkg.  This will create a folder named flood_monitor_16 inside your ROS workspace, ~/catkin_ws/
- **IMPORTANT**: I recommend using the package name flood_monitor_16 since the launch files will be calling this project with that name. Otherwise you will have to change the project calls in the launch files. 
- Download my project files from the flood_monitor_16 folder on this Github repository into the flood_monitor_16 folder in your ROS workspace.
- Next, build the catkin workspace Run catkin_make
- 
