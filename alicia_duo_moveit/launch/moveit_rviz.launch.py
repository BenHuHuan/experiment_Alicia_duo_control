from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('alicia_duo_moveit'),
                'config/alicia_rviz.rviz'
            ]),
            description='Path to the RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        )
    ])

