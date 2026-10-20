from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='vx300s',
            description='Interbotix arm model (e.g. vx300s, wx250s).',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description='Use the DYNAMIXEL simulator instead of the real robot.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='Launch RViz alongside the driver.',
        ),
    ]

    xsarm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                # the driver behind driver_guard: Ctrl+C parks the arm before it stops
                FindPackageShare('xsarm_scripts'),
                'launch',
                'arm.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('use_sim'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    joint_gui = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='xsarm_scripts',
                executable='joint_gui',
                output='screen',
            ),
        ],
    )

    return LaunchDescription(declared_arguments + [xsarm_control, joint_gui])
