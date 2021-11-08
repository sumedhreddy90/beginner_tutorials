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
