#!/bin/bash

#
# probe for the ROS2 distro
#
distros=('foxy' 'galactic' 'humble')
for distro in "${distros[@]}"
do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
	source /opt/ros/${distro}/setup.bash
	break
    fi
done

echo "found ros version: ${ROS_VERSION} distro: ${ROS_DISTRO}"

vcs import --recursive < src/basalt_ros/basalt_ros.repos
vcs import --recursive < src/basalt_wrapper/basalt_wrapper.repos

# build and test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && colcon test && colcon test-result --verbose


