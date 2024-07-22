import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv

class FakeDetectionPublisher(Node):
    def __init__(self):
        super().__init__('fake_detection_publisher')
        self.publisher = self.create_publisher(String, 'fake_detection', 10)
        self.get_logger().info('Fake Detection Publisher has been started')

    def publish_fake_detection(self, detection_type):
        msg = String()
        msg.data = detection_type
        self.publisher.publish(msg)
        self.get_logger().info(f'Published fake detection: {detection_type}')

def load_fmea_data():
        csv_file_path = 'src/recovery_pipeline/recovery_pipeline/resources/fmea.csv'
        causes = []
        with open(csv_file_path, 'r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                cause = row['Cause']
                causes.append(cause)
        return causes

def main(args=None):
    node = FakeDetectionPublisher()
    while rclpy.ok():
        try:
            options = load_fmea_data()
            detection_type = input('Enter the type of fake detection - options are: ' + ', '.join(options) + '\n')
            node.publish_fake_detection(detection_type)
        except KeyboardInterrupt:
            print('Shutting down...')
            node.destroy_node()
            break

if __name__ == '__main__':
    rclpy.init()
    main()
    rclpy.shutdown()