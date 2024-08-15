# Example usage: . quick_env_setup.sh altered_map

#!/bin/bash

export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=`pwd`/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo $1.launch.py &
sleep 5
# Ensure map is chosen when running this bash script
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/behavior_trees/behavior_trees/resources/$1.yaml


