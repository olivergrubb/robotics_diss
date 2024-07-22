#!/bin/bash

sudo colcon build

source install/setup.bash
. /usr/share/gazebo/setup.sh

ros2 run behavior_trees test_behavior


