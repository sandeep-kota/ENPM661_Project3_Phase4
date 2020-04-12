<h1 align="center"> ENPM661 Phase4 

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Introduction

This is a simulation of a A* algorithm with constraints.


## Dependencies

This package has been tested in a system with following dependencies.
- Ubuntu 16.04 LTS
- ROS-Kinetic distro
- Gazebo 7+
- Turtlebot simulator

## Build Instructions

1) To install ROS-Kinetic follow the steps mentioned in the official website (http://wiki.ros.org/kinetic/Installation/Ubuntu)

2) To install gazebo for ros kinetic run the following sommand in a terminal
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
``` 
3) To install `turtlebot_sumulator` package, run the following commands in a terminal window.
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
``` 

4) After installing the required dependencies run the following commands to download this project.
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/sandeep-kota/ENPM661_Project3_Phase4.git
cd ../ 
catkin_make
```

## Run Instructions

1) To launch the program and simulation world in the custom world run the following commands in a terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch planning turtlebot_astar.launch 
# Set rosbagRecorder argument value to true to record a bag file for 15 seconds
```
This will spawn a turtlebot robot in the given world file. 
2) To start the astar algoritm run the following node by typing the following command in a new terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/ENPM661_Project3_Phase4/results/
rosrun planning astar.py
```

## LICENSE
This project is released under the 3-clause BSD License.
