import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ollie/ros2_ws/install/robot_vacuum_auto_recovery'
