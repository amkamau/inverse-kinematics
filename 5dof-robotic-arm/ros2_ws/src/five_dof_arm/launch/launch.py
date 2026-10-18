
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('five_dof_arm'),
        'urdf',
        'arm.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='five_dof_arm',
            executable='xarm_state',
            name='xarm_state',
            output='screen',
        ),
        Node(
            package='five_dof_arm',
            executable='servo_to_joint_state',
            name='servo_to_joint_state',
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

    ])
