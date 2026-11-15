
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class XArmTeleop(Node):

    def __init__(self):
        super().__init__('xarm_teleop')

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        self.step = 0.05
        self.tolerance = 0.02

        # Maximum time allowed for a movement.
        self.movement_timeout = 5.0

        self.joint_min = [-2.7, -1.5, -1.5, -1.5, -1.5, 0.0]
        self.joint_max = [ 2.7,  1.5,  1.5,  1.5,  1.5, 1.5708]

        self.current_positions = {}
        self.state_received = False
        self.last_state_time = None

        # Tracks the one outstanding movement.
        self.active_joint = None
        self.active_target = None
        self.command_time = None
        self.last_feedback_time = None

        self.key_map = {
            'q': ('joint1',  1),
            'a': ('joint1', -1),
            'w': ('joint2',  1),
            's': ('joint2', -1),
            'e': ('joint3',  1),
            'd': ('joint3', -1),
            'r': ('joint4',  1),
            'f': ('joint4', -1),
            't': ('joint5',  1),
            'g': ('joint5', -1),
            'y': ('joint6',  1),
            'h': ('joint6', -1),
        }

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.command_pub = self.create_publisher(
            JointTrajectory,
            '/teleop/joint_commands',
            10
        )

        self.check_timer = self.create_timer(
            0.02,
            self.check_movement
        )

        self.get_logger().info('XArm teleop initialized.')

    def joint_state_callback(self, msg):

        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in self.joint_names):
            return

        self.current_positions = {
            name: positions[name]
            for name in self.joint_names
        }

        self.state_received = True
        self.last_state_time = time.monotonic()

        if self.active_joint is not None:
            self.last_feedback_time = self.last_state_time

    def process_key(self, key):

        if key == 'x':
            return False

        if key not in self.key_map:
            return True

        if not self.state_received:
            self.get_logger().warning(
                'Waiting for valid joint states.'
            )
            return True

        if self.active_joint is not None:
            self.get_logger().warning(
                'Robot is still moving. Wait for completion.'
            )
            return True

        # Reject stale feedback.
        if time.monotonic() - self.last_state_time > 0.5:
            self.get_logger().warning(
                'Joint-state feedback is stale. Command rejected.'
            )
            return True

        joint_name, direction = self.key_map[key]

        current_position = self.current_positions[joint_name]

        index = self.joint_names.index(joint_name)

        target_position = current_position + (
            direction * self.step
        )

        target_position = max(
            self.joint_min[index],
            min(self.joint_max[index], target_position)
        )

        if abs(target_position - current_position) < 1e-6:
            self.get_logger().info(
                f'{joint_name} is already at its limit.'
            )
            return True

        self.publish_command(joint_name, target_position)

        return True

    def publish_command(self, joint_name, target_position):

        msg = JointTrajectory()

        # Send all six positions, preserving the current
        # positions of joints that are not being moved.
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        msg.joint_names = [joint_name]
        point.positions = [target_position]

        point.time_from_start.sec = 1

        msg.points.append(point)

        # Set tracking state before publishing.
        self.active_joint = joint_name
        self.active_target = target_position
        self.command_time = time.monotonic()
        self.last_feedback_time = self.command_time

        self.command_pub.publish(msg)

        self.get_logger().info(
            f'Command sent: {joint_name} -> '
            f'{target_position:.3f} rad'
        )

    def check_movement(self):

        if self.active_joint is None:
            return

        now = time.monotonic()

        # Check whether feedback is still arriving.
        if now - self.last_feedback_time > 0.5:
            self.get_logger().error(
                'Joint-state feedback lost. '
                'Movement tracking stopped.'
            )
            self.clear_active_command()
            return

        # Check movement timeout.
        if now - self.command_time > self.movement_timeout:
            self.get_logger().error(
                f'Movement timeout: {self.active_joint}'
            )
            self.clear_active_command()
            return

        current = self.current_positions[self.active_joint]

        error = abs(current - self.active_target)

        if error <= self.tolerance:
            self.get_logger().info(
                f'Movement complete: {self.active_joint} '
                f'at {current:.3f} rad'
            )
            self.clear_active_command()

    def clear_active_command(self):

        self.active_joint = None
        self.active_target = None
        self.command_time = None
        self.last_feedback_time = None


def main(args=None):

    rclpy.init(args=args)

    node = XArmTeleop()

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    print('\n========== XARM TELEOP ==========')
    print('Q/A : Joint 1 +/-')
    print('W/S : Joint 2 +/-')
    print('E/D : Joint 3 +/-')
    print('R/F : Joint 4 +/-')
    print('T/G : Joint 5 +/-')
    print('Y/H : Joint 6 +/-')
    print('X   : Exit')
    print('=================================')

    running = True

    try:
        tty.setraw(sys.stdin.fileno())

        while rclpy.ok() and running:

            rclpy.spin_once(node, timeout_sec=0.01)

            readable, _, _ = select.select(
                [sys.stdin], [], [], 0.01
            )

            if readable:
                keys = sys.stdin.read(32)

                for key in keys.lower():
                    if not node.process_key(key):
                        running = False
                        break

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
