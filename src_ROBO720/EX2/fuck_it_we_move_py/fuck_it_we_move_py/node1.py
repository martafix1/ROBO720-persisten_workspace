import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class FrankaMover(Node):
    def __init__(self):
        super().__init__('franka_mover')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/JntMoveCommands',
            10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Give the publisher some time to set up
        time.sleep(1)

        
        

    def timer_callback(self):
        # Create a message
        msg = Float64MultiArray()

        # Target positions for all 7 joints (in radians)
        # You can change these values!
        msg.data = [0.2, -0.6, 0.2, -1.5, 0.2, 1.0, 0.5]

        self.get_logger().info('Sending joint command...')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mover = FrankaMover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()
