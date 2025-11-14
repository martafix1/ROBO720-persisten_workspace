#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from pathing_py6.srv import GenerateTrajectory  # Replace with your actual package name

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory


class PlanTrajectoryServer(Node):

    def __init__(self):
        super().__init__('plan_trajectory_server')
        self.srv = self.create_service(
            GenerateTrajectory,
            'generate_trajectory',
            self.plan_trajectory_callback
        )
        self.get_logger().info('GenerateTrajectory service is ready.')

    def plan_trajectory_callback(self, request, response):
        # Log the incoming request
        # self.get_logger().info(f'Received planning request in {request.space_type} space.')

        # Dummy implementation
        response.success = False
        response.points = 0
        response.message = 'Planning not implemented yet.'

        # Prepare empty trajectories
        response.joint_trajectory = JointTrajectory()
        response.task_trajectory = MultiDOFJointTrajectory()

        # TODO: Implement real planning logic here based on space_type
        if request.joint_space:
            # request.start_joint_state: JointState
            # request.goal_joint_state: JointState
            self.get_logger().info('Planning in joint space...')
            # Do joint space planning...
        if request.task_space:
            # request.start_joint_state: MultiDOFJointTrajectory
            # request.goal_joint_state: MultiDOFJointTrajectory
            self.get_logger().info('Planning in task space...')
            # Do task space planning...
        
        

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanTrajectoryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
