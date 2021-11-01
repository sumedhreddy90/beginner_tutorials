# ROS Publisher Subscriber

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
catkin_make
source devel/setup.bash
Run the executable:
$rosrun beginner_tutorials talker
$rosrun beginner_tutorials listener
```
