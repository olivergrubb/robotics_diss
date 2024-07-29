from std_msgs.msg import String
from rclpy.node import Node

class PlaceRobotUprightErrorResponder(Node):
    def __init__(self):
        super().__init__('place_robot_upright_error_responder')
        self.publisher = self.create_publisher(String, 'error_responder', 10)

    def respond(self):
        self.get_logger().error('MANUAL ACTION REQUIRED: Please place robot upright. Press ENTER when robot is ready to proceed' )
        input()
        self.get_logger().info('Attempting to continue...')
        error_responder_msg = String()
        error_responder_msg.data = 'manual_finished'
        self.publisher.publish(error_responder_msg)
    
    def get_solution_name(self):
        return 'MANUAL: place_robot_upright'