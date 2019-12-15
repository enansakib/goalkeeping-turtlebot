# A Goalkeeping Turtlebot implemented in ROS
This is a part of my small effort in creating some basic projects implemented in ROS. As I am totally new at learning ROS, feel free to give constructive comments. If you are absolutely new at learning ROS, it might help you. You may use the codes as-is. 

## Introduction 


## Pre-requisites
- Python 2 
- Turtlebot 2 (should work with Turtlebot 3 as well, however, you may have to change Topic names)
- [ZED camera](https://www.stereolabs.com/docs/ros/)

## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/enansakib/goalkeeping-turtlebot.git
cd ~/catkin_ws
catkin_make
```

## Usage
```
roslaunch turtlebot_bringup minimal.launch
roslaunch zed_wrapper zed.launch
roslaunch goalkeeping-turtlebot turtlebotgk.launch
```

## Demo
![demo.gif](demo/demo.gif)


## Details
The code inside the `src` folder already has necessary comments to understand what's going on. 


### Note
This is implemented on Ubuntu 16.04, ROS Kinetic Kame, Turtlebot 2.

## Reference
1. https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
2. http://wiki.ros.org/ROS/Tutorials
3. https://www.stereolabs.com/docs/ros/
4. http://wiki.ros.org/cv_bridge
3. 
