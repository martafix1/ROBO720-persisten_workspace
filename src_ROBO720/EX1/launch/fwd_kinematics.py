
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='KinematicsEX1',
            executable='rnd_configurable',
            name='rnd_gen1',
            output='screen',
            parameters=[{
                'min': 0.0,
                'max': 10.0,
                'period_ms': 1000
            }],
            remappings=[('/random_number', '/input1')]
        ),

        Node(
            package='KinematicsEX1',
            executable='rnd_configurable',
            name='rnd_gen2',
            output='screen',
            parameters=[{
                'min': -5.0,
                'max': 5.0,
                'period_ms': 500
            }],
            remappings=[('/random_number', '/input2')]
        ),

        Node(
            package='KinematicsEX1',
            executable='node1',
            name='kinematics_node1',
            output='screen',
        ),

    ])
