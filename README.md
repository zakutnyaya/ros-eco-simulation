# About the project
This is a ROS simulation of a robot prototype that sorts garbage using computer vision. The simulation made it possible to debug the behavior of the prototype at the stages of garbage detection and interaction with detected objects.

# How it works
![This is an image](/assets/ecobot.gif)

The simulation is placed in a virtual world with three axes: x (red line), y (green line), and z (blue line). It consists of three main blocks:
- **Circular belt** that rotates clockwise around the z-axis.
- **Manipulator** that can move along y- and z-axes and captures objects from the table with its four fingers.
- **Camera** whose field of view is marked by rays. 
The fourth but invisible component is **a computer vision component** that detects objects on the table and calculates current and future real-world coordinates of the detected object.
All components of the system interact with each other via ROS infrastructure.

The task of the system is to remove garbage (a cube) from the table. It's done in several simple steps:
1. The table with the cube starts spinning. The camera starts taking pictures. 
2. Camera frames are transmitted to a garbage detector (neural network) via ROS infrastructure.
3. Detector outputs bounding boxes for a detected object.
4. Real-world coordinates of the detected object are calculated. Those are coordinates at time t0 (when the photo was taken).
5. Calculate the time t1 when the object crosses the x-axis, along which the robot arm moves. Calculate the y-coordinate of the x-axis intersection point.
6. Send the y-coordinate and the time t1 of object arrival on the x-axis to robot controllers.
7. Table stops rotating at time t1. The robot arm starts moving towards the object at time t1.
8. The robot arm captures the object and removes it from the table.

Implementation details can be found in the source code which is accompanied by detailed comments. the structure of the repo follows a ROS-like style.


# Installation (Ubuntu)
If you want to test the system or adapt the system to your task, please follow the installation steps:
1. Follow [this tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) and install ROS Noetic Full Desktop version with Gazebo. Gazebo is necessary for simulation visualization.
2. Install ROS package with trajectory controllers for the robot arm:
```
sudo apt-get install ros-noetic-joint-trajectory-controllers
```
3. Install all the necessary Python packages:
```
cd ros-eco-simulation
poetry install —only-main
```
4. Build workspace:
```
catkin_make
source devel/setup.bash
```
5. Call the following launch files to start the simulation:
```
roslaunch launch/simulation.launch
roslaunch launch/detection.launch
roslaunch launch/robot.launch
```
