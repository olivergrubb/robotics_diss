# robotics_diss

## Environment Setup from Clean Ubuntu 22.04 Install

*Skip installs as necessary*

### Prerequisites
- Ubuntu 22.04 (Jammy)
- ROS2 (Humble) - [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Turtlebot3 + Gazebo + Nav2 Installation

```bash
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
```

**Note:** Ensure file structure is `~/ros2_ws/src/...` and startup scripts are at `~/ros2_ws` level.

```bash
sudo apt install ros-humble-py-trees
sudo apt install ros-humble-py-trees-ros-interfaces
sudo apt install ros-humble-py-trees-ros
```

### Bug Fixes

1. Fix nav2 bug:
   Change `robot_model_type: "differential"` to `"nav2_amcl::DifferentialMotionModel"` in `/opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml`

2. Fix file paths to models in `/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models`:
   Replace all `/home/ollie/Documents/Blender Models/` with `/$PATH_TO_ROS_WORKSPACE$/src/turtlebot3_simulations/turtlebot3_gazebo/models`

### Build

Make sure you're still in `~/ros2_ws`, then run:
```bash
colcon build --symlink-install
```

## Main Setup Scripts

*Run in separate terminals*

### Environment (Gazebo + RVIZ)

Three options for different maps:

1. Basic map:
   ```bash
   . quick_env_setup.sh altered_map
   ```
2. AWS small house map:
   ```bash
   . quick_env_setup.sh aws_small_house_map
   ```
3. Bookstore map:
   ```bash
   . quick_env_setup.sh bookstore_map
   ```

**Note:** When terminating the environment, ensure you close both RVIZ and Gazebo. If you use Ctrl+C in the terminal to exit, run `fg` to see the background process and Ctrl+C out of it too.

### Main Vacuum Planner

Three options for different maps:

1. Basic map:
   ```bash
   . vacuum_planner.sh altered_map -8.6894 7.69215 0.4 -0.7 9 1 1
   ```
2. AWS small house map:
   ```bash
   . vacuum_planner.sh aws_small_house_map -0.36147 -0.62154 -0.1 -0.6 9 11 20
   ```
3. Bookstore map:
   ```bash
   . vacuum_planner.sh bookstore_map -0.17053 -0.35185 0.0 0.6 9 15 17
   ```

### Recovery Pipeline

```bash
. recovery_pipeline_start.sh
```

**Note:** When terminating the recovery pipeline, ensure you close both the error detector and error responder. If you use Ctrl+C in the terminal to exit, run `fg` to see the background process and Ctrl+C out of it too.

## Invoking an Error

To inject a 'fake' error:
Run the Python file `fake_detection_pub.py` in `/src/recovery_pipeline/recovery_pipeline/resources`.

To trigger an 'actual' error:
Manipulate the Gazebo environment.

## Additional Information

The `long_cable` model in the simulation is VERY computationally intensive and will likely slow down the simulation significantly on less powerful computers. Remove the cable for increased performance.
