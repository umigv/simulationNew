#!/bin/zsh
source devel/setup.zsh
roslaunch atom world_no_rviz.launch 2> >(grep -v TF_REPEATED_DATA)