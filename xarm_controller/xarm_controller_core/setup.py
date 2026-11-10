
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xarm_controller_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'hidapi==0.15.0',
    ],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='XArm controller package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xarm_position_initializer = xarm_controller_core.xarm_position_initializer:main',
            'trajectory_processor = xarm_controller_core.trajectory_processor:main',
            'trajectory_server = xarm_controller_core.trajectory_server:main',
            'servo_to_joint_state = xarm_controller_core.servo_to_joint_state_node:main',
            'xarm_state = xarm_controller_core.xarm_state_node:main',
        ],
    },
)

