# ROS wrappers for Basalt

This repo has ROS1 and ROS2 wrappers for the
[Basalt VIO/mapper library](https://gitlab.com/VladyslavUsenko/basalt/)

## Supported systems

ROS Melodic and Eloquent on Ubuntu 18.04LTS 

## How to install


Clone this repo, and all the submodules along with it:

    cd top_of_your_workspace
	mkdir src
	cd src
    git clone --recursive https://github.com/berndpfrommer/basalt_ros.git

### Building under ROS2 (Eloquent)

    cd top_of_your_workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

### Building under ROS1 (Melodic)

    cd top_of_your_worskapce
    catkin config -DCMAKE_BUILD_TYPE=Release
	catkin build


## How to use

Instructions for use:

- For ROS1: see the [ROS1 wrapper repo](https://github.com/berndpfrommer/basalt_ros1)
- For ROS2: see the [ROS2 wrapper repo](https://github.com/berndpfrommer/basalt_ros2)



