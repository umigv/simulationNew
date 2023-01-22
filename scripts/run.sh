#!/bin/zsh
cd ~/robotics_ws
catkin build
source devel/setup.zsh
roslaunch atom world.launch 2> >(grep -v TF_REPEATED_DATA)