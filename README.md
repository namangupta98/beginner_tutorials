# ROS Services, Logging and Launch files
[![License: GNU](https://img.shields.io/badge/License-GNU-green.svg)](https://opensource.org/licenses/GPL-3.0)

## Overview

This repository is made for course ENPM808X  which contains beginner tutorials of ROS C++. This project depends on the following:

    ROS kinetic : To install ROS refer [ROS installation page](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    catkin : To install catkin refer [catkin installation page](http://wiki.ros.org/catkin?distro=kinetic#Installing_catkin)

## Build Instructions

We are assuming that the dependencies are met, so we now follow the below mentioned commands on terminal to clone this repository:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source ~/devel/setup.bash
cd src
git clone https://github.com/namangupta98/beginner_tutorials.git
cd ..
catkin_make
```

## Run Instructions

Now, we use launch file to run. In a new terminal, type 

```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials launcher.launch
```
## Service

To change the output string message, open a new terminal and type

```
cd catkin_ws
source devel/setup.bash
rosservice call /changeBaseOutputString "Hi"
```
