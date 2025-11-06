from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    arm_controllers_share = FindPackageShare("arm_controllers")
    franka_gazebo_share = FindPackageShare("franka_gazebo")
    franka_description_share = FindPackageShare("franka_description")

    # Controller YAML
    task_space_controller_yaml = PathJoinSubstitution(
        [arm_controllers_share, "config", "task_space_controller.yaml"]
    )

    # Load URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([franka_description_share, "urdf", "effort_panda_arm.urdf.xacro"]),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            PathJoinSubstitution([franka_gazebo_share, "config", "franka_controllers.yaml"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, robot_description],
        output="screen"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[task_space_controller_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    task_space_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["task_space_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description_content,
            "-name", "panda_arm",
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 1 empty.sdf"}.items(),
    )

    return LaunchDescription([
        gz_launch_description,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        task_space_controller_spawner,
        gz_spawn_entity,
    ])
