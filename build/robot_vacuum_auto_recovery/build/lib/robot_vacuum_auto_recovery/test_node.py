from irobot_create_msgs.msg import InterfaceButtons, LightringLeds

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class TestNode(Node):
    lights_on_ = False

    def __init__(self):
        super().__init__('test_node')

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)
    

    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        if create3_buttons_msg.button_1.is_pressed:
            self.get_logger().info('Button 1 Pressed!')
            # Publish to the cmd_vel topic to make the robot go forward
            cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            twist_msg = Twist()
            twist_msg.linear.x = 0.5  # Set the linear velocity to 0.5 m/s
            twist_msg.angular.z = 0.0  # Set the angular velocity to 0.0 rad/s
            cmd_vel_publisher.publish(twist_msg)



def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

