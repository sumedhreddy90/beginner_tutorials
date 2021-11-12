# ROS Publisher Subscriber
[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)

## Description
This is a simple ROS project to program publisher node to publish a custom string message and listen the message

### Dependencies:

- ROS Melodic for Ubuntu 18.04
- The source code is built with C++ 11 features and compiled with g++ compiler
- built system with catkin and CMake

## Create Catkin workspace and clone repo:
```bash
mkdir catkin_ws/src
cd catkin_ws
git clone --recursive https://github.com/sumedhreddy90/beginner_tutorials.git
```
## Week 10
## Build source:

```bash

$catkin_make

$source devel/setup.bash

Run the executable:

Start publisher node:
$rosrun beginner_tutorials talker

Start subscriber node:
$rosrun beginner_tutorials listener

To run launch file
//This command will launch both nodes publisher aswellas subscriber

$roslaunch beginner_tutorials beginner.launch frequency:=<desired-frequency>

To run service
$rosservice call /Service "<desired-custom-message>"

To invoke rqt logger and console 

$rosrun rqt_console rqt_console
$rosrun rqt_logger_level rqt_logger_level
```
## Week 11 

## Rosbag Instructions

1) To launch the subscriber publisher node with rosbag node to collect data, use flag `rosbagRecorder:=true` flag at the end of roslaunch command.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials talker_listener.launch frequency:=20 rosbagRecorder:=true
```
2) To replay the rosbag data collected, run the following command while the listener node is running in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/beginner_tutorials/Results/
rosbag play recorder.bag 
```

## TF2 Instructions

Below are the tools that can be used to debug the tf2 frames being broadcasted by the nodes.

1) To view the tf2 frames being broadcasted by the `talker` node, run the following command. This will show a graph of how the frames are connected.
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/beginner_tutorials/results/
rosrun tf2_tools view_frames.py
evince frames.pdf
```

2) To view the transform between the `/world` frame and the /`talk` frame, use `tf_echo`.
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun tf tf_echo world talk
```

## Test Running Instructions

1) To run the unit test cases, we can use the `catkin_make run_tests_beginner_tutorials` command.
```
cd ~/catkin_ws/
catkin_make run_tests_beginner_tutorials
``` 
## Result

<img width="1440" alt="RQTLogger-with-Service" src="https://user-images.githubusercontent.com/24978535/140796334-8a016ead-0be9-4a0d-863e-1047d12c6e57.png">
