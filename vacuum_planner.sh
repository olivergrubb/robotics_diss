#!/bin/bash

colcon build --symlink-install

source install/setup.bash
. /usr/share/gazebo/setup.sh

ros2 run behavior_trees vacuum_planner --ros-args -p map_name:='altered_map' -p start_position_x:=-8.6894 -p start_position_y:=7.69215 -p x_offset:=0.4 -p y_offset:=-0.7 -p map_resolution:=9 -p start_location:="[1, 1]"
