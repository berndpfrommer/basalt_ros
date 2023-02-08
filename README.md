# Basalt ROS: ROS2 wrapper for Basalt VIO

This repo has a ROS2 wrapper for the
[Basalt VIO library](https://gitlab.com/VladyslavUsenko/basalt/). ROS1
is no longer supported. The original Basalt repo is still available
under the "deprecated_ros1_ros2" branch.

## Supported systems

Continuous integration testing on:

- ROS2 galactic on Ubuntu 20.04 (focal)
- ROS2 humble on Ubuntu 22.04 (jammy)

## How to install

### Clone this repo

    cd top_of_your_workspace
	mkdir src  # (if not already there)
    git clone https://github.com/berndpfrommer/basalt_ros.git ./src/basalt_ros
	# now fetch the dependencies (including the basalt repository etc)
    vcs import --recursive < src/basalt_ros/basalt_ros.repos

### Building

    cd top_of_your_workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

If this step fails it is probably because some Ubuntu packages are
missing. Just install them one by one until you succeed.

## How to use

... to be done ...

## License

This software is issued under the Apache License Version 2.0.
