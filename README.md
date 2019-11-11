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
## Inspect TF Frames

The tf frame is verified and inspected by rqt_tf_tree and tf_echo package. Open a new terminal and type the following commands

```
cd catkin_ws
source devel/setup.bash
rosrun tf tf_echo /world /talk
rosrun tf view_frames
evince frames.pdf
rosrun rqt_tf_tree rqt_tf_tree
```
The output of the above is generated in the a pdf titles frames.pdf, which is included in the Results sub directory.

## Rosbag

The rosbag recording is disabled by default so when you type

```
roslaunch beginner_tutorials launcher.launch
```
Only launch file is executed. To generate a new rosbag file type 

```
roslaunch beginner_tutorials launcher.launch rosbagEnable:=true
```
A bag file will be created in Results directory which is recorded for 15 seconds. To play the ROS bag file, terminate the talker and listener nodes, and run the following command after launching roscore in a new terminal.

```
rosrun beginner_tutorials listener
```
Open new terminal, and navigate to Results directory.

```
cd cakin_ws/src/beginner_tutorials/Results
rosbag play rec.bag
```
You will see that the message stored in bagfile is running as talker and listener can hear the talker in the listener terminal. o get information about the rosbag file you can type
```
rosbag info rec.bag
```

## Tests

Close and terminate everything including rosmaster. In a new terminal, switch to the ROS workspace and build the tests. Type

```
cd catkin_ws
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

The above command will also run the tests after few seconds. You can also type the following command to run all test using launch file.

```
rostest beginner_tutorials unitTest.launch
```
The above can only be executed after the tests are built.
