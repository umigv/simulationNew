# Simulation workspace

This repository includes the simulation code for UMARV.

## Contents

- [Prerequisites](#prerequisites)
- [Set Up](#set-up-workspace)
- [Atom Bot](#atom-robot)
- [Control Bots](#control-the-bot)

## Prerequisites
  
- ROS (`$ sudo apt-get install ros-noetic-desktop-full`)
- Xacro (`$ sudo apt-get install ros-noetic-xacro`)
- Gazebo (`$ sudo apt-get install ros-noetic-gazebo-ros`)

## Set up workspace

```bash
git clone -b updated https://github.com/umigv/simulationNew.git simulation_ws
cd simulation_ws
git submodule update --init --recursive
catkin build
source devel/setup.sh
```

## Atom Robot

```bash
roslaunch atom world.launch
```

## Control the bot

Use `teleop_twist_keyboard`

```bash
rosrun atom teleop_twist_keyboard.py
```
