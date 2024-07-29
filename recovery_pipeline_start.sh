export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
ros2 run recovery_pipeline error_detection &
ros2 run recovery_pipeline error_responder 


