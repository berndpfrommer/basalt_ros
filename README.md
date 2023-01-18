# Basalt ROS: wrappers for Basalt VIO

This repo has a combined ROS1/ROS2 wrapper for the
[Basalt VIO/mapper library](https://gitlab.com/VladyslavUsenko/basalt/)

## Supported systems

Continuous integration testing on:

- ROS1 noetic on Ubuntu 20.04 (focal)
- ROS2 galactic on Ubuntu 20.04 (focal)

## How to install

### Clone this repo

    cd top_of_your_workspace
	mkdir src
    git clone --branch fresh_start	https://github.com/berndpfrommer/basalt_ros.git ./src/basalt_ros
	# now fetch the dependencies (including the basalt repository etc)
    vcs import --recursive < src/basalt_ros/basalt_ros.repos

### Building under ROS1

    cd top_of_your_worskapce
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
	catkin build

### Building under ROS2

    cd top_of_your_workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

## How to use

... to be done ...

## License

This software is issued under the Apache License Version 2.0.
