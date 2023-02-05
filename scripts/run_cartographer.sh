#!/bin/zsh
source devel/setup.zsh
rosrun xacro xacro src/atom/urdf/atom.xacro > src/atom/urdf/atom.urdf
roslaunch cartographer_ros my_robot.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')