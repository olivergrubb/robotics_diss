export TURTLEBOT3_MODEL=waffle
sudo colcon build
source install/setup.bash
ros2 run recovery_pipeline error_detection &
ros2 run recovery_pipeline error_responder 


