
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='xarm_controller_core',
            executable='xarm_position_initializer',
            output='screen',
        ),

        Node(
            package='xarm_controller_core',
            executable='trajectory_server',
            output='screen',
        ),

        Node(
            package='xarm_controller_core',
            executable='trajectory_processor',
            output='screen',
        ),

        Node(
            package='xarm_controller_core',
            executable='xarm_state',
            output='screen',
        ),

        Node(
            package='xarm_controller_core',
            executable='servo_to_joint_state',
            output='screen',
        ),

    ])
