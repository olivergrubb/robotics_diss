# Example usage: 
# For basic map: . quick_env_setup.sh altered_map
# For aws small house map: . quick_env_setup.sh aws_small_house_map
# For bookstore map: . quick_env_setup.sh bookstore_map

#!/bin/bash

export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=`pwd`/src/turtlebot3_simulations/turtlebot3_gazebo/models
cd src/turtlebot3_simulations/turtlebot3_gazebo/
ros2 launch turtlebot3_gazebo $1.launch.py &
cd ../../../
sleep 5
# Ensure map is chosen when running this bash script
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/behavior_trees/behavior_trees/resources/$1.yaml


