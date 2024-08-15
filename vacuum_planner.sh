# Example usage . vacuum_planner.sh altered_map -8.6894 7.69215 0.4 -0.7 9 "[1, 1]"

#!/bin/bash

colcon build --symlink-install

source install/setup.bash
. /usr/share/gazebo/setup.sh

# Select all the parameters when running the vacuum_planner node
ros2 run behavior_trees vacuum_planner --ros-args -p map_name:=$1 -p start_position_x:=$2 -p start_position_y:=$3 -p x_offset:=$4 -p y_offset:=$5 -p map_resolution:=$6 -p start_location:=$7
