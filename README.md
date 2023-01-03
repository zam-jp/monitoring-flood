# Control Strategy for Monitoring Flood and Its Verification in Gazebo

## Summary
- This is the title for my graduation research.
- The objective was to find an inundation in an unkown area using multiple drones.
- A control strategy was developed using an adaptive controller based on Function Approximation Technique Based Immersion and Invariance (FATII), a flood detection algorithm based on image segmentation, and a distribution algorithm based on Centroidal Voronoi Tessellation.
- Gazebo with ROS was used for the simulator environment.
- A ROS package called Hector Quadrotor was used for simulating the drones (17 altogether: 16 for flood detection and 1 for the purpose of overhead view recording).
- The 3D world was created using BLENDER with a GIS (Geographic Information System) plugin and then exported to Gazebo in Collada format.

## How to Install and Run the Simulation
### Prerequisite
- Have ROS Noetic installed.
- - Install from http://wiki.ros.org/noetic/Installation/Ubuntu
- Have Hector Quadrotor installed.
- Have Python 3 (or above) installed.

### My Environment
- Ubuntu version: 20
- ROS version: Noetic 1.15.11
- Gazebo version: 11.5.1 (This should comes together with the ROS version installed)
- Python version: 3.8.5
