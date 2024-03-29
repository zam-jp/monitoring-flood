# Control Strategy for Monitoring Flood and Its Verification in ROS Gazebo

## Summary
- This is the title for my graduation research.
- The objective was to find an inundation in an unkown area using multiple drones.
- A control strategy was developed using an adaptive controller based on Function Approximation Technique Based Immersion and Invariance (FATII), a flood detection algorithm based on image segmentation, and a distribution algorithm based on Centroidal Voronoi Tessellation.
- Gazebo with ROS (Robot Operating System) was used for the simulator environment.
- A ROS package called Hector Quadrotor was used for the drone models (17 altogether: 16 for flood detection and 1 for the purpose of overhead view recording).
- The 3D world was created using BLENDER with a GIS (Geographic Information System) plugin and then exported to Gazebo in Collada format.

## How to Install and Run the Simulation
### Prerequisite
- Have ROS Noetic installed. (Install from http://wiki.ros.org/noetic/Installation/Ubuntu)
  - Dependency package: timed_roslaunch.
- Have Hector Quadrotor installed. (Install from https://github.com/RAFALAMAO/hector-quadrotor-noetic)
  - **IMPORTANT**: For this project, a modified quadrotor model was used. The required files have been uploaded into the hector_quadrotor folder. The sub-folders in the hector_quadrotor folder reflects where you need to copy the files to.
- Have Python 3 (or above) installed.
  - Required libraries: panda, shapely, opencv, geovoronoi, etc.

### My Environment
- Ubuntu version: 20.04
- ROS version: Noetic 1.15.11
- Gazebo version: 11.5.1 (This should comes together with the ROS version installed)
- Python version: 3.8.5

### Installing the Project
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

- Warning message in the terminal from Python about using deprecated array interface. This is related to the shapely package update patch.
  - Where to fix: /home/.local/lib/python3.8/site-packages/descartes/patch.py
  - What to do: Open the Python file with a text editor and look for the lines containing *t.exterior*. Change it to *t.exterior.coords*

- Graphics for the world are not loading.
  - Where to fix: Look at all .world files.
  - What to do: Check and change the file paths to the images as needed. By default, I have used relative paths, but for some reason, sometimes the computer could not find the resources. I found that replacing the relative paths with absolute paths can fix this.

### Run the Simulation
Follow this steps to run the simulation.
- STEP 1: First of all, open a new terminal and start roscore by typing **roscore** in the command-line. This terminal should remain open with roscore running. Type CTRL-C to stop roscore.

- STEP 2: Next, open another terminal and go to the catkin workspace directory (~/catkin_ws/). To launch the ROS package for the project use the following command in the command-line, **roslaunch [package] [filename.launch]**. For this project it should look like this, **roslaunch flood_monitor_16 flood_monitor.launch**. This will launch the ROS Gazebo simulator. You will see the simulator already loaded with the 3D world. Notice the red drones spawning on the green square off to the side of the 3D world, which represents the launch platform. Wait until all 17 drones have finished spawning before going to STEP 3.

- STEP 3: Lastly, again open another terminal and go to the src directory (~/catkin_ws/src/). Then type, **.\flood_monitor.sh**, in the command-line, to start the simulation. You will see a bunch of new terminals open up for each drone and then the drone will start taking-off. A window showing video from the drone's camera will pop-up. When all the drones are in their starting position, a final window will pop-up to signify that the algorithm is executing. The final window will be displaying the centroidal Voronoi tessellation and the density function plot. To stop the simulation, you have to type CTRL-C for every open terminal.

### The worlds created for this project
- Case 1
![world01](https://user-images.githubusercontent.com/84988870/226152539-aeaa133b-9058-49b9-b04f-923ffee698f8.png)

- Case 2
![world02](https://user-images.githubusercontent.com/84988870/226152548-fb414750-9216-45ad-aacd-c3b36787e3f9.png)

- Case 3
![world03](https://user-images.githubusercontent.com/84988870/226152586-8a30a197-e412-472c-b203-35590831c442.png)

- Case 4
![world04](https://user-images.githubusercontent.com/84988870/226152595-d705b200-46c8-49a9-abc7-26e4810afe10.png)

- Case 5
![world05](https://user-images.githubusercontent.com/84988870/226152600-a0428202-50af-4903-873b-b98bc9a82106.png)

- Case 6
![world06](https://user-images.githubusercontent.com/84988870/226152603-8c0751c6-ceac-4fab-b71d-53c42fef66e3.png)

### Example results: The CVT plot (left) and the Density plot (right) from Case 1.
- Note: In the CVT plot, the black circles represents the drones, and the red squares represents the centroids.
- At t = 0s
![FIG_0000](https://user-images.githubusercontent.com/84988870/226152774-08174e0d-e4b9-4bc7-a456-9078f60bdae3.png)

- At t = 10s
![FIG_0009](https://user-images.githubusercontent.com/84988870/226152781-2026cedb-884f-4015-b090-102c0e12e0b6.png)

- At t = 15s
![FIG_0014](https://user-images.githubusercontent.com/84988870/226152785-dc65d2f4-1541-424b-9c45-b896c93f0810.png)

- At t = 60s
![FIG_0054](https://user-images.githubusercontent.com/84988870/226152788-735bbdcd-eed0-460f-be92-a4eca9d86de0.png)

