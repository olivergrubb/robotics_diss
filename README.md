# robotics_diss

Environment setup from clean ubuntu 22.04 install - skip installs as nessesary:

Ubuntu 22.04 (Jammy)
ROS2 (Humble) - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Turtlebot3 + Gazebo + Nav2 install:

sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

source ~/.bashrc
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-colcon-common-extensions

mkdir -p ~/ros2_ws

sudo apt install git

cd ~/ros2_ws

git clone --no-checkout https://github.com/olivergrubb/robotics_diss.git .
git checkout

(Ensure file structure is ~/ros2_ws/src/... and startup scripts are at ~/ros2_ws level)

sudo apt install ros-humble-py-trees
sudo apt install ros-humble-py-trees-ros-interfaces
sudo apt install ros-humble-py-trees-ros

Fix nav2 bug - change robot_model_type: "differential" to "nav2_amcl::DifferentialMotionModel" in /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml

Fix file paths to models in /ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
(Replace all /home/ollie/Documents/Blender Models/ with /$PATH_TO_ROS_WORKSPACE$/src/turtlebot3_simulations/turtlebot3_gazebo/models)

Make sure still in ~/ros2_ws
colcon build --symlink-install

---------- MAIN SETUP SCRIPTS (Run in separate terminals) ----------

Environment (Gazebo + RVIZ):
. quick_env_setup.sh

Main Vaccum Planner:
. vacuum_planner.sh

Recovery Pipeline:
. recovery_pipeline_start.sh


---------- TO INVOKE AN ERROR ----------

Run the python file fake_detection_pub.py in /src/recovery_pipeline/recovery_pipeline/resources to inject a 'fake' error.

Manipulate the Gazebo environment to trigger an 'actual' error.



---------- ADDITIONAL INFORMATION ----------

The long_cable model in the simulation is VERY computationally intensive and will likely slow down the simulation significantly on less powerful computers. Remove the cable for increased performance.
