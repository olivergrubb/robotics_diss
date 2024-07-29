#!/bin/bash

colcon build --symlink-install

source install/setup.bash
. /usr/share/gazebo/setup.sh

ros2 run behavior_trees vacuum_planner
