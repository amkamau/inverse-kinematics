
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'xarm_controller_hardware'
    )

    urdf_file = os.path.join(
        package_dir,
        'test',
        'test_robot.urdf'
    )

    controllers_file = os.path.join(
        package_dir,
        'config',
        'ros2_controllers.yaml'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
        output='both'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file
        ],
        output='both'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='both'
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_spawner
    ])
