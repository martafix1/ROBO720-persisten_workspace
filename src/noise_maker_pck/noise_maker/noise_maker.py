import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class NoisePublisher(Node):
    def __init__(self):
        super().__init__('noise_publisher')
        self.publisher_ = self.create_publisher(Float32, 'noise', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        noise_value = random.uniform(-1.0, 1.0)
        msg = Float32()
        msg.data = noise_value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing noise value: {noise_value:.4f}')

def main(args=None):
    rclpy.init(args=args)
    noise_publisher = NoisePublisher()
    rclpy.spin(noise_publisher)
    noise_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
