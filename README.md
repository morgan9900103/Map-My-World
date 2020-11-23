# go_chase_it

## Overview

In this project I created two ROS package inside ```catkin_ws/src```: the ```drive_bot``` and the ```ball_chaser```.

1. ```drive_bot```:

- Created a ```my_robot``` ROS package to hold my robot, the white ball, and the world.
- Designed a differential drive robot with the URDF including two sensors: a lidar and a camera. 
- Added a white-colored ball to your Gazebo world.
- The ```world.launch``` file will launch the world with white-colored ball and robot.

2. ```ball_chaser```

- Created a ```ball_chaser``` ROS package to hold C++ nodes.
- ```drive_bot``` C++ node that will provide a ```ball_chaser/command_robot``` service to drive the robot by controlling its linear x and angular z velocities. The service will publish to the wheel joints and return back the reauested velocities.
- ```process_image``` C++ node will reads the robot's camera image, analyzes it to determine the presence and position of a white ball. It will request a service via a client to drive the robot towards it.
- The ```ball_chaser.launch``` will run both ```drive_bot``` and the ```process_image``` nodes.

## Project Description

Directory Structure

```
.catkin_ws/src
├── ball_chaser
│   ├── CMakeLists.txt
│   ├── include
│   │   └── ball_chaser
│   ├── launch
│   │   └── ball_chaser.launch
│   ├── package.xml
│   ├── src
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   └── srv
│       └── DriveToTarget.srv
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
├── my_ball
│   ├── model.config
│   └── model.sdf
├── my_robot
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── meshes
│   │   └── hokuyo.dae
│   ├── package.xml
│   ├── urdf
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── worlds
│       ├── empty.world
│       └── world.world
└── README.md
```

## Run the project

- Clone this repository
- Open the repository and make

```
cd ~/catkin_ws
catkin_make
```
- Launch
```
roslaunch my_robot world.launch
```
  Open a new terminal and type
```
roslaunch ball_chaser ball_chaser.launch
```
- Visualize
  Open a new terminal and type
```
rosrun rqt_image_view rqt_image_view
```

