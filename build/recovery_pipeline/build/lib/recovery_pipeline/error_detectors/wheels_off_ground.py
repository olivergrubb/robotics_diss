from nav_msgs.msg import Odometry

class WheelsOffGroundErrorDetector:
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info(f'Loaded error detector')
        self.subscriber = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.wheels_off_ground = False
        self.last_odom_msg = None
    
    def odom_callback(self, msg):
        self.last_odom_msg = msg

    def detect(self):
        if self.last_odom_msg is None:
            self.node.get_logger().info('No odometry message received yet')
            return False, ''
        elif float(self.last_odom_msg.pose.pose.orientation.x) < -0.4 or float(self.last_odom_msg.pose.pose.orientation.x) > 0.4 or float(self.last_odom_msg.pose.pose.orientation.y) < -0.4 or float(self.last_odom_msg.pose.pose.orientation.y) > 0.4:
            return True, 'wheels_off_ground'
        else:
            return False, ''