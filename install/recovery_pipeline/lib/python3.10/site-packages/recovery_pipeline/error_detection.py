import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from recovery_pipeline.error_detectors.error_detectors import ALL_ERROR_DETECTORS

class ErrorDetectorNode(Node):
    def __init__(self):
        super().__init__('error_detector')
        self.error_detectors = []
        self.publisher = self.create_publisher(String, 'error_topic', 10)
        self.responder_subscriber = self.create_subscription(String, 'error_responder', self.responder_callback, 10)
        self.fake_detection_subscriber = self.create_subscription(String, 'fake_detection', self.fake_detection_callback, 10)
        self.timer = self.create_timer(1.0, self.check_errors)
        self.load_error_detectors()
        self.responder_active = False
        self.fake_detection_type = None

    def load_error_detectors(self):
        for detector_class in ALL_ERROR_DETECTORS:
            self.error_detectors.append(detector_class(self))

    def fake_detection_callback(self, msg):
        self.fake_detection_type = msg.data

    def check_errors(self):
        for detector in self.error_detectors:
            error, error_type = detector.detect()
            if error and not self.responder_active:
                self.publish_error(error_type)
            elif self.fake_detection_type is not None:
                self.publish_error(self.fake_detection_type)
                self.fake_detection_type = None

    def publish_error(self, error_type):
        msg = String()
        msg.data = error_type
        self.publisher.publish(msg)

    def responder_callback(self, msg):
        if msg.data == 'responding':
            self.responder_active = True
        else:
            self.responder_active = False
            

def main(args=None):
    rclpy.init(args=args)
    error_detector = ErrorDetectorNode()
    rclpy.spin(error_detector)
    error_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()