#!/bin/zsh
source devel/setup.zsh
roslaunch cartographer_ros my_robot.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')