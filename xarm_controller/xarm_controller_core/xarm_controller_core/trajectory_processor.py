
import rclpy

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32MultiArray

import xarm


class TrajectoryProcessor(Node):
    def __init__(self):
        super().__init__('trajectory_processor')

        self.arm = None

        # Servo IDs: joint1 -> servo 1, ..., joint6 -> servo 6
        # self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.servo_ids = [6, 5, 4, 3, 2, 1]

        # Joint name to position-array index
        self.joint_map = {'joint1': 0, 'joint2': 1, 'joint3': 2, 'joint4': 3, 'joint5': 4, 'joint6': 5}

        self.raw_min = [0, 0, 0, 0, 0, 0]
        self.raw_max = [1000, 1000, 1000, 1000, 1000, 1000]

        self.angle_min = [-2.7, -1.5, -1.5, -1.5, -1.5, 0.0]
        self.angle_max = [2.7, 1.5, 1.5, 1.5, 1.5, 1.5708]

        # Maintain the latest complete six-joint position state.
        # Initial values assume all joints start at zero radians.
        self.current_positions = [self.angle_to_raw(0.0, i) for i in range(6)]

        # Three independent trajectory subscriptions.
        self.arm_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/raw_trajectory_1_4',
            self.trajectory_callback,
            10
        )
        self.wrist_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/raw_trajectory_5',
            self.trajectory_callback,
            10
        )
        self.gripper_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/raw_trajectory_6',
            self.trajectory_callback,
            10
        )
        self.virtual_positions_pub = self.create_publisher(
            Int32MultiArray,
            '/xarm/virtual_positions_raw',
            10
        )

        self.connection_timer = self.create_timer(0.5, self.connect_arm)

        self.get_logger().info('Trajectory processor started. Waiting for USB controller.')

    def connect_arm(self):
        if self.arm is not None:
            return

        try:
            self.arm = xarm.Controller('USB')
            self.get_logger().info('XArm USB controller connected.')

        except Exception as exc:
            self.arm = None
            # self.get_logger().warning(f'XArm USB controller unavailable. Retrying... ({exc})')

    def disconnect_arm(self):
        if self.arm is not None:
            close = getattr(self.arm, 'close', None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
        self.arm = None

    def angle_to_raw(self, angle, index):

        angle_min = self.angle_min[index]
        angle_max = self.angle_max[index]

        raw_min = self.raw_min[index]
        raw_max = self.raw_max[index]

        # Keep the angle within the configured range.
        angle = max(angle_min, min(angle_max, angle))

        # Normalize the angle.
        fraction = ((angle - angle_min) / (angle_max - angle_min))

        # Convert normalized position into servo units.
        raw = (raw_min + fraction * (raw_max - raw_min))

        return round(raw)

    def trajectory_callback(self, msg):

        previous_time_ms = 0.0

        joint_names = msg.joint_names

        # Validate joint names.
        unknown_joints = [
            name for name in joint_names
            if name not in self.joint_map
        ]

        if unknown_joints:
            self.get_logger().error(f'Unknown joints received: {unknown_joints}')
            return

        if len(joint_names) != len(set(joint_names)):
            self.get_logger().error(f'Duplicate joint names received: {joint_names}')
            return

        self.get_logger().info(f'Processing trajectory for joints: {joint_names}')

        for point in msg.points:

            if len(point.positions) != len(joint_names):
                self.get_logger().error(
                    f'Joint count mismatch: {len(joint_names)} names, '
                    f'{len(point.positions)} positions.'
                )
                continue

            current_time_ms = (
                point.time_from_start.sec * 1000
                + point.time_from_start.nanosec / 1e6
            )

            duration = round(current_time_ms - previous_time_ms)
            previous_time_ms = current_time_ms

            commands = []

            # Update only the joints included in this trajectory.
            for joint_name, position in zip(joint_names, point.positions):

                index = self.joint_map[joint_name]
                servo_id = self.servo_ids[index]

                raw_position = self.angle_to_raw(position, index)

                # Preserve the latest position of all other joints.
                self.current_positions[index] = raw_position

                commands.append((servo_id, raw_position))

            self.get_logger().info(f'Servo commands: {commands}')

            # Publish the complete six-servo position state.
            virtual_msg = Int32MultiArray()
            # virtual_msg.data = self.current_positions.copy()
            virtual_msg.data = self.current_positions[::-1]

            self.virtual_positions_pub.publish(virtual_msg)

            # Execute only the servos addressed by this trajectory.
            if self.arm is not None:
                try:
                    self.arm.setPosition(commands, wait=True, duration=max(duration, 1))

                except Exception as exc:
                    self.get_logger().error(f'Arm execution failed: {exc}')
                    self.disconnect_arm()

    def destroy_node(self):
        self.disconnect_arm()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
