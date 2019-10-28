# ROS Beginner Tutorial
[![License: GNU](https://img.shields.io/badge/License-GNU-green.svg)](https://opensource.org/licenses/GPL-3.0)

## Overview

This repository is made for course ENPM808X  which contains beginner tutorials of ROS C++ in publisher and subscriber node from [ROS website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

    Talker (src/talker.cpp): Publisher Node
    Listener (src/listener.cpp): Subscriber Node

This project depends on the following:

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

Open three seperate terminals. In terminal-1 type

```
roscore
```
In terminal-2 type

```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```
In terminal-3 type

```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```
