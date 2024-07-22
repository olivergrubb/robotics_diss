import py_trees
import time
from geometry_msgs.msg import Twist
import numpy as np
import math
class FollowWall(py_trees.behaviour.Behaviour):
    def __init__(self, node, speed=0.3, wall_side="right", wall_offset=0.7, corner_offset=0.7, name="Follow Wall"):
        super(FollowWall, self).__init__(name)
        self.node = node
        self.speed = speed
        self.lidar_msg = None
        self.in_corner = False
        self.wall_side = wall_side
        self.wall_offset = wall_offset
        self.corner_offset = corner_offset
        self.prev_error = 0
        self.previous_error_angle = 0
        self.previous_error_dist = 0
        self.previous_time = time.time()
        self.integral_dist = 0
        self.integral_angle = 0

    def initialise(self):
        self.node.get_logger().info("Starting follow wall")
        while self.lidar_msg is None:
            self.node.get_logger().info("Waiting for lidar data")
            self.lidar_msg = self.node.most_recent_lidar_data
            time.sleep(0.1)

    def update(self):
        self.node.get_logger().info("Update")
        self.lidar_msg = self.node.most_recent_lidar_data
        if not self.in_corner:
            ranges = np.array(self.lidar_msg.ranges)
            
            min_range = np.inf
            min_index = -1
            for i, range in enumerate(ranges):
                if float(range) < min_range:
                    min_range = range
                    min_index = i
        
            self.node.get_logger().info("Min distance: {} - Min Index: {}".format(min_range, min_index))

            twist = Twist()
            twist.linear.x = self.speed
            dist_error = self.wall_offset - min_range
            angle_error = self.calculate_error_angle(min_index, self.wall_side)

            self.node.get_logger().info("Angle Error: {}".format(angle_error))
            self.node.get_logger().info("Distance Error: {}".format(dist_error))

            twist.angular.z = self.compute_angular_velocity(dist_error, angle_error)

            # Has approached a corner - stop
            if (min_index < 20 or min_index > 340) and min_range < self.corner_offset:
                self.node.get_logger().info("In corner - stopping")
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.in_corner = True
            # Facing the correct direction - move forward
            
            self.node.get_logger().info("Following wall")        
            self.node.cmd_vel_publisher.publish(twist)

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        twist = Twist()
        twist.linear.x = 0.0
        self.node.cmd_vel_publisher.publish(twist)
        self.node.get_logger().info("Stopped following wall")

    def compute_angular_velocity(self, error_dist, error_angle):
        Kp_dist=0.3
        Kd_dist=0.1
        Ki_dist=0.01
        Kp_angle=0.5
        Kd_angle=0.1
        Ki_angle=0.01

        current_time = time.time()
        delta_time = current_time - self.previous_time
        
        # Calculate the distance error
        self.integral_dist += error_dist * delta_time
        derivative_dist = (error_dist - self.previous_error_dist) / delta_time if delta_time > 0 else 0
        
        # Calculate the angle error (desired angle is typically 0 for parallel alignment)
        self.integral_angle += error_angle * delta_time
        derivative_angle = (error_angle - self.previous_error_angle) / delta_time if delta_time > 0 else 0
        
        # PID control output
        angular_velocity_dist = (Kp_dist * error_dist + 
                                 Kd_dist * derivative_dist + 
                                 Ki_dist * self.integral_dist)
                                 
        angular_velocity_angle = (Kp_angle * error_angle + 
                                  Kd_angle * derivative_angle + 
                                  Ki_angle * self.integral_angle)
        
        # Combine distance and angle control efforts
        angular_velocity = angular_velocity_dist + angular_velocity_angle
        
        if self.wall_side == 'left':
            angular_velocity = -angular_velocity

        # Update previous error and time
        self.previous_error_dist = error_dist
        self.previous_error_angle = error_angle
        self.previous_time = current_time

        return angular_velocity
    def calculate_error_angle(self, current_angle, wall_side):
        if wall_side == 'left':
            desired_angle = 90
            # Calculate error angle
            error_angle = desired_angle - current_angle
            # Normalize to [-180, 180]
            error_angle = (error_angle + 180) % 360 - 180

        elif wall_side == 'right':
            desired_angle = 270
            error_angle = -(desired_angle - current_angle)
        else:
            raise ValueError("wall_side must be 'left' or 'right'")


        # Convert to radians
        error_angle_radians = math.radians(error_angle)

        return error_angle_radians