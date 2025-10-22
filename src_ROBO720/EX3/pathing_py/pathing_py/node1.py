import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
import math


# This node publishes sinusoidal joint trajectory points—positions, velocities,
# and accelerations—for 7 joints at 100 Hz on the /requested_traj_point topic.


class JointTrajectoryPointPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_point_publisher')

        # Parameters
        self.num_joints = 7
        self.frequency = 100.0  # Hz
        self.dt = 1.0 / self.frequency

        # Publisher (single JointTrajectoryPoint)
        self.publisher_ = self.create_publisher(JointTrajectoryPoint, '/requested_traj_point', 10)

        # Timer to publish at 100Hz
        self.timer = self.create_timer(self.dt, self.publish_point)

        # Time tracking
        self.start_time = self.get_clock().now()
        self.get_logger().info("JointTrajectoryPoint publisher started.")

    def generate_point(self, t):
        """
        Generate a simple sinusoidal joint trajectory point.
        """
        point = JointTrajectoryPoint()

        # Parameters for sine wave
        freq = 0.1  # Hz
        amp = 0.3   # radians

        # Joint-wise sine patterns
        point.positions = [amp * math.sin(2 * math.pi * freq * t + i) for i in range(self.num_joints)]
        point.velocities = [2 * math.pi * freq * amp * math.cos(2 * math.pi * freq * t + i) for i in range(self.num_joints)]
        point.accelerations = [
            - (2 * math.pi * freq) ** 2 * amp * math.sin(2 * math.pi * freq * t + i) for i in range(self.num_joints)
        ]

        return point

    def publish_point(self):
        now = self.get_clock().now()
        elapsed_time = now - self.start_time
        t = elapsed_time.nanoseconds * 1e-9  # Convert to float seconds

        point_msg = self.generate_point(t)
        self.publisher_.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()