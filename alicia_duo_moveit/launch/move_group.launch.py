from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    load_robot_description = LaunchConfiguration('load_robot_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='load_robot_description',
            default_value='true',
            description='Whether to load robot_description parameter here'
        ),

        # Only load robot_description if instructed
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(load_robot_description),
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
            output='screen'
        ),

        # Move group node
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                # Example: Add your MoveIt config yaml or controllers here
            ],
        )
    ])

