from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_load = LaunchConfiguration('camera_load')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('alicia_duo_descriptions'),
            'urdf/alicia_gripper_ros2control.urdf'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument('camera_load', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            parameters=[
                robot_description,
                PathJoinSubstitution([
                    FindPackageShare('alicia_duo_moveit'),
                    'config/ros2_controllers.yaml'
                ])
            ],
            output='screen',
        ),


        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_controller', 'arm_pos_controller', 'gripper_controller'],
                    output='screen'
                )
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('alicia_duo_moveit'),
                    'launch/move_group.launch.py'
                ])
            ),
            launch_arguments={'load_robot_description': 'false'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('alicia_duo_moveit'),
                    'launch/moveit_rviz.launch.py'
                ])
            ),
            launch_arguments={
                'rviz_config': PathJoinSubstitution([
                    FindPackageShare('alicia_duo_moveit'),
                    'config/alicia_rviz.rviz'
                ])
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('easy_handeye'),
                    'launch/publish.launch.py'
                ])
            ),
            condition=IfCondition(camera_load),
            launch_arguments={'namespace_prefix': 'Alicia_usb_handeyecalibration'}.items()
        ),
    ])

