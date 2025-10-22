import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
import time
import numpy as np

# This Node will listen arm's end effector point and arm_pose_publisher's trajectory and compare them


class TrajectoryEEListener(Node):
    def __init__(self):
        try:
            super().__init__('trajectory_ee_listener')

            self.trajectory_msg = None
            self.arm_msg = None

            self.trajectory_subscriber_ = self.ceate_subscription(_, _, self.callback_trajectory_listener, 10)
            self.arm_subscriber_ = self.ceate_subscription(_, _, self.callback_arm_listener, 10)
            self.create_timer(1.0, self.timer_callback)

        except Exception as e:
            print(f" Error during TrajectoryEEListener init: {e}")

    # Listens arm's desired position
    def callback_trajectory_listener(self, msg):
        self.trajectory_msg = msg

    # Listens arm's current position
    def callback_arm_listener(self, msg):
        self.arm_msg = msg

    # compares desired and current positions
    def timer_callback(self,):
        twist_ = Twist()

        #TODO: Comparison between ee point and trajectory point
        if self.arm_msg is not None and self.trajectory_msg is not None:
            x_err = twist_.diff( _, _)

    

def main(args=None):
    rclpy.init(args=args)
    print("Creating node...")
    node = TrajectoryEEListener()
    print("Node created")
    try:
        print("Spinning")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
        
    finally:
        print("Stopping")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("Done")
    print("Well Done")


# if __name__ == '__main__':
#     main()