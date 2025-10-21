import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory,  MultiDOFJointTrajectoryPoint
import math
from geometry_msgs.msg import Transform, Twist,PoseStamped
from tf_transformations import quaternion_from_euler
from franka_cc_3.srv import ComputeIK




class JointSpaceArbiter(Node):
    def __init__(self):
        super().__init__('joint_space_arbiter')


        self.gracefull_init()

        # Publisher (single JointTrajectoryPoint)
        self.publisher_ = self.create_publisher(JointTrajectoryPoint, '/requested_traj_point', 10)

        # Timer to publish at 20Hz
        self.timer = self.create_timer(0.5, self.next_step)

        # Time tracking
        self.start_time = self.get_clock().now()
        self.get_logger().info("requested_traj_point publisher started.")


        # Create the subscriber
        self.taskspaceObjectiveSubscription = self.create_subscription(
            MultiDOFJointTrajectory,
            '/taskspace_objective',
            self.taskspaceObjectiveSubscription_callback,
            10  # QoS history depth
        )
        self.taskspaceObjectiveSubscription  # prevent unused variable warning

        self.get_logger().info('Subscribed to /taskspace_objective')



        self.computeIKClient = self.create_client(ComputeIK, '/compute_ik')

        while not self.computeIKClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        self.computeIKRequest = ComputeIK.Request()
        self.get_logger().info('Subscribed to /compute_ik service')


    def gracefull_init(self):

        self.objective_point = MultiDOFJointTrajectoryPoint()

        # Position + Orientation as Transform
        transform = Transform()
        transform.translation.x = 0.0
        transform.translation.y = 0.0
        transform.translation.z = 1.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 0.0  # Identity quaternion

        # Velocity as Twist
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0

        # Acceleration as Twist
        acceleration = Twist()
        acceleration.linear.x = 0.0
        acceleration.linear.y = 0.0
        acceleration.linear.z = 0.0
        acceleration.angular.x = 0.0
        acceleration.angular.y = 0.0
        acceleration.angular.z = 0.0

        self.objective_point.transforms.append(transform)
        self.objective_point.velocities.append(velocity)
        self.objective_point.accelerations.append(acceleration)


    def send_IK_request(self, objective_point):
        # Fill the PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.5
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.computeIKRequest.target_pose = pose
        self.get_logger().info(f"Sending request")
        # Call the service
        future = self.computeIKClient.call_async(self.computeIKRequest)
        rclpy.spin_until_future_complete(self, future,timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"IK Success! Joint Positions: {response.solution.position}")
            else:
                self.get_logger().warn(f"IK Failed: {response.error_message}")
        else:
            self.get_logger().error("Service call failed")



    def taskspaceObjectiveSubscription_callback(self, msg):
        # Called every time a new message is received on /taskspace_objective
        # self.get_logger().info(f'Received trajectory with {len(msg.points)} points.')
        
        if(len(msg.points) < 1 ):
            self.get_logger.error("Recieved taskspace_objective has no points. ")

        self.objective_point = msg.points[0]
        self.get_logger().info(f'Updated objective point {self.objective_point} .')
        


    def next_step(self):

        # point_msg = JointTrajectoryPoint()


        self.send_IK_request("kek")

        # point_msg = self.generate_point(t)
        # self.publisher_.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointSpaceArbiter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()