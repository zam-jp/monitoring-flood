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
- Ubuntu version: 20.04
- ROS version: Noetic 1.15.11
- Gazebo version: 11.5.1 (This should comes together with the ROS version installed)
- Python version: 3.8.5

### Installing My Project
Before continuing, if you are not familiar with ROS, I recommend doing the ROS beginner tutorial (http://wiki.ros.org/ROS/Tutorials). Next, make sure you have all the prerequisites installed, then you can begin installing my project. This is the way I have tried to install it and it worked. We will be mostly using commond-lines on the terminal.
- Create a ROS package flood_monitor_16 using the ROS command catkin_create_pkg.  This will create a folder named flood_monitor_16 inside your ROS workspace, also called catkin workspace, located at ~/catkin_ws/src/
- **IMPORTANT**: I recommend using the package name flood_monitor_16 since the launch files will be calling this project with that name. Otherwise you will have to change the project calls in the launch files. 
- Download my project files from the flood_monitor_16 folder on this Github repository into the flood_monitor_16 folder in your catkin workspace.
- Next, build the catkin workspace. Make sure you are in the directory ~/catkin_ws/ and not ~/catkin_ws/src/ or any other directory. Then run catkin_make from the command-line.
- Finally, we need to add the workspace to the ROS environment. Go to the directory ~/catkin_ws/devel/ and run setup.bash from the command-line.

You should be able to launch the project now, if there is no issues. You can tell if there are issues, for example, the graphics for the world are not loading, or there are error and warning messages in the terminal.

### Known Potential Issues
These are some issues that I have ran into when I was working on this project.
- Controller update period for Hector Quadrotor is faster than Gazebo simulator.
  - Where to fix: ~/hector_quadrotor_gazebo/urdf/quadrotor_controller.gazebo.xarco
  - What to do: Change controller period value from 0.01 to 0.0125

- Warning message in the Python terminal about using deprecated array interface. This is related to the shapely package update patch.
  - Where to fix: /home/.local/lib/python3.8/site-packages/descartes/patch.py
  - What to do: Open the Python file with a text editor and look for the lines containing *t.exterior*. Change it to *t.exterior.coords*

- Graphics for the world are not loading.
  - Where to fix: Look at all .world files.
  - What to do: Check and change the file paths to the images as needed. By default, I have used relative paths, but for some reason, sometimes the computer could not find the resources. I found that replacing the relative paths with absolute paths can fix this.

### Run the Simulation
Follow this steps to run the simulation.
- STEP 1: First of all, open a new terminal and start roscore by typing *roscore* in the command-line. This terminal should remain open with roscore running. Type CTRL-C to stop roscore.
- STEP 2: Next, open another terminal and go to the catkin workspace directory (~/catkin_ws/). To launch the ROS package for the project use the following command in the command-line, *roslaunch [package] [filename.launch]*. For this project it should look like this, *roslaunch flood_monitor_16 flood_monitor.launch*. This will launch the ROS Gazebo simulator. You will see the simulator already loaded with the 3D world. Notice the red drones spawning on the green square off to the side of the 3D world, which represents the launch platform. Wait until all 17 drones have finished spawning before going to STEP 3.
- STEP 3: Lastly, again open another terminal and go to the src directory (~/catkin_ws/src/). Then type, *.\flood_monitor.sh*, in the command-line, to start the simulation. You will see a bunch of new terminals open up for each drone and then the drone will start taking-off. A window showing video from the drone's camera will pop-up. When all the drones are in their starting position, a final window will pop-up to signify that the algorithm is executing. The final window will be displaying the centroidal Voronoi tessellation and the density function plot. To stop the simulation, you have to type CTRL-C for every open terminal.
