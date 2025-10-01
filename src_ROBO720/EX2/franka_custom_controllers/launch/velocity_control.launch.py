#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    franka_description_path = get_package_share_directory('franka_description')
    franka_gazebo_path = get_package_share_directory('franka_gazebo')
    custom_controller_path = get_package_share_directory('franka_custom_controllers')
    
    # Get URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [franka_description_path, "urdf", "panda_arm.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=false",
            " ",
            "fake_sensor_commands:=false",
            " ",
            "use_gazebo:=true",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Load controller configuration
    controller_config = PathJoinSubstitution(
        [custom_controller_path, "config", "velocity_controller.yaml"]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'panda',
            '-x', '0',
            '-y', '0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )
    
    # Load and start controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'franka_velocity_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_velocity_controller],
            )
        ),
    ])