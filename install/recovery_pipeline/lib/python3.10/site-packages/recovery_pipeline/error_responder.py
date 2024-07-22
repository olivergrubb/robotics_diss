import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from recovery_pipeline.error_responders.error_responders import ALL_ERROR_RESPONDERS

class ErrorResponder(Node):
    def __init__(self):
        super().__init__('error_responder')
        self.subscription = self.create_subscription(
            String,
            'error_topic',
            self.error_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'error_responder', 10)
        self.fmea_data = self.load_fmea_data()

    def error_callback(self, msg):
        error_message = msg.data
        solution = self.get_solution_from_fmea(error_message)
        if solution is not None:
            for responder in ALL_ERROR_RESPONDERS:
                if responder.get_solution_name(self) == solution:
                    error_responder_msg = String()
                    error_responder_msg.data = 'responding'
                    self.publisher.publish(error_responder_msg)
                    responder_instance = responder()
                    responder_instance.respond()
        else:
            self.get_logger().info(f'No solution found for error {error_message}')

    def get_solution_from_fmea(self, cause):
        for row in self.fmea_data:
            if row['Cause'] == cause:
                return row['Solution']
        return None
    
    def load_fmea_data(self):
        csv_file_path = 'src/recovery_pipeline/recovery_pipeline/resources/fmea.csv'
        with open(csv_file_path, 'r') as file:
            csv_reader = csv.DictReader(file)
            fmea_data = []
            for row in csv_reader:
                fmea_data.append(row)
        return fmea_data
    
def main(args=None):
    rclpy.init(args=args)
    error_responder = ErrorResponder()
    rclpy.spin(error_responder)
    error_responder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()