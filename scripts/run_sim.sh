#!/bin/zsh
source devel/setup.zsh
roslaunch atom world.launch 2> >(grep -v TF_REPEATED_DATA)