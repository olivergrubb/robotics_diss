#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class VacuumProgressMarker(Node):
    def __init__(self):
        super().__init__('vacuum_progress_marker')
        self.marker_pub = self.create_publisher(Marker, 'vacuum_progress_marker', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_limiter = 20
        self.marker_counter = 0
        self.marker_id = 0

        self.get_logger().info("TurtleBot Marker Node has been started")

    def odom_callback(self, msg):
        if self.marker_counter == self.marker_limiter:
            self.marker_counter = 0
            # Extract position from odometry message
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation

            map_fixed_frame_x_offset = 0
            map_fixed_frame_y_offset = 0

            position.x += map_fixed_frame_x_offset
            position.y += map_fixed_frame_y_offset

            # Create and publish the marker
            marker = self.create_marker(position, orientation)
            self.marker_pub.publish(marker)

            self.get_logger().info(f"Published marker at position: {position.x}, {position.y}, {position.z}")
        else:
            self.marker_counter += 1

    def create_marker(self, position, orientation):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vacuum_markers"
        marker.id = self.marker_id
        self.marker_id += 1

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = position
        
        # Set marker pose with orientation
        marker.pose.orientation = orientation
        
        # Set marker scale (size)
        marker.scale.x = 0.45  # Diameter of the cylinder
        marker.scale.y = 0.45
        marker.scale.z = 0.01  # Height of the cylinder

        # Set marker color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the lifetime of the marker (0 means infinite)
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = VacuumProgressMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
