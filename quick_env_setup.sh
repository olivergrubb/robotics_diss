export TURTLEBOT3_MODEL=waffle
sudo colcon build
source install/setup.bash
. /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_gazebo house_interior.launch.py &
sleep 5
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/altered_map.yaml

