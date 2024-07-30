import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.publisher_ = \
            self.create_publisher(Float64MultiArray, 
                '/base_velocity_controller/commands', 
                10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        # Set desired controls for the joints
        msg.data = [1.5 if index % 2 == 0 else 0.75 for index in range(8)]
        msg.data = [data * 1 for data in msg.data]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    control_publisher = ControlPublisher()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
