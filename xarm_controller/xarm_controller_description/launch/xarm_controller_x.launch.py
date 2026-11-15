#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    use_gui = LaunchConfiguration('use_gui')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('xarm_controller_description'),
            'urdf',
            'xarm_controller_x',
            'xarm_controller_x.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_mock_hardware:=',
        'True',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('xarm_controller_description'),
        'rviz',
        'xarm_controller.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Run joint state publisher gui node',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(urdf_file,value_type=str)}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
    ])
