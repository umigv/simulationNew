# Simulation Workspace

This repository includes the simulation code for UMARV.

## Contents

- [Prerequisites](#prerequisites)
- [Clone the Workspace](#clone-the-workspace)
- [Set Up Cartographer](#set-up-cartographer)
- [Build the Workspace](#build-the-workspace)
- [Launch the Robot](#launch-the-robot)
- [Control the Robot](#control-the-robot)

## Prerequisites
  
- ROS (`sudo apt-get install ros-noetic-desktop-full`)
- Xacro (`sudo apt-get install ros-noetic-xacro`)
- Gazebo (`sudo apt-get install ros-noetic-gazebo-ros`)

## Clone the Workspace

```bash
git clone --recursive https://github.com/umigv/simulationNew.git simulation_ws
cd simulation_ws
```

## Set up Cartographer

Comment out line 46 (`<!-- <depend>libabsl-dev</depend> -->`) in Cartographer package.xml as per [https://github.com/cartographer-project/cartographer_ros/issues/1726](https://github.com/cartographer-project/cartographer_ros/issues/1726)

```bash
# Install prerequisites
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build
sudo apt-get install -y liblua5.3-dev python3-sphinx libeigen3-dev
sudo apt-get install -y stow

# Install Cartographer dependencies
src/cartographer/scripts/install_proto3.sh
src/cartographer/scripts/install_debs_cmake.sh
src/cartographer/scripts/install_abseil.sh
src/cartographer/scripts/install_ceres.sh

# Install ROS dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## Build the Workspace

```bash
catkin build
source devel/setup.bash
```

## Launch the Robot

```bash
# Launch with sensors in RVIZ
roslaunch marvin world.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')

# Launch with no RVIZ (for Cartgrapher)
roslaunch marvin world_no_rviz.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')
```

## Launch Cartographer

```bash
roslaunch marvin cartographer.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')
```

## Control the Robot

Use `teleop_twist_keyboard`

```bash
rosrun marvin teleop_twist_keyboard.py
```
