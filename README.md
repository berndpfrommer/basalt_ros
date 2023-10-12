# INACTIVE: Basalt ROS: ROS2 wrapper for Basalt VIO


NOTE: due to lack of time this repo is currently unsupported, meaning
issues will be left unaddressed. Sorry!

# ----------------------------


This repo has a ROS2 wrapper for the
[Basalt VIO library](https://gitlab.com/VladyslavUsenko/basalt/). ROS1
is no longer supported. The original Basalt repo is still available
under the "deprecated_ros1_ros2" branch.

## Supported systems

Continuous integration testing on:

- ROS2 galactic on Ubuntu 20.04 (focal)
- ROS2 humble on Ubuntu 22.04 (jammy)

## How to install

### Install dependencies

The following apt packages must be installed on top of the ROS2 base
image:
```
sudo apt-get install libbz2-dev
sudo apt-get install ros-humble-ament-cmake-clang-format
```

### Clone this repo

    cd top_of_your_workspace
	mkdir src  # (if not already there)
    git clone https://github.com/berndpfrommer/basalt_ros.git ./src/basalt_ros
	# now fetch the dependencies (including the basalt repository etc)
	# (this can take a while, patience)
    vcs import --recursive < src/basalt_ros/basalt_ros.repos
    vcs import --recursive < src/basalt_wrapper/basalt_wrapper.repos

### Building

    cd top_of_your_workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

If this step fails it is probably because some Ubuntu packages are
missing. Just install them one by one until you succeed.

## How to use

... to be done ...

## License

This software is issued under the Apache License Version 2.0.
