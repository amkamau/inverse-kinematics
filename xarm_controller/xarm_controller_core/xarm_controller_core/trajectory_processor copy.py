
import rclpy

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32MultiArray

import xarm


class TrajectoryProcessor(Node):
    def __init__(self):
        super().__init__('trajectory_processor')

        self.arm = None
        self.servo_ids = ([1, 2, 3, 4],[5],[6])

        self.raw_min = [0, 0, 0, 0, 0, 0]
        self.raw_max = [1000, 1000, 1000, 1000, 1000, 1000]

        self.angle_min = [-2.7, -1.5, -1.5, -1.5, -1.5, 0.0]
        self.angle_max = [2.7, 1.5, 1.5, 1.5, 1.5, 1.5708]

        self.trajectory_sub = self.create_subscription( JointTrajectory, '/raw_trajectory', self.trajectory_callback, 10)
        self.virtual_positions_pub = self.create_publisher( Int32MultiArray, '/xarm/virtual_positions_raw', 10)
        self.connection_timer = self.create_timer( 0.5, self.connect_arm )

        self.get_logger().info('Trajectory processor started. Waiting for USB controller.' )

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

        for point in msg.points:

            current_time_ms = ((point.time_from_start.sec * 1000) + (point.time_from_start.nanosec / 1e6))

            duration = round(current_time_ms - previous_time_ms)
            previous_time_ms = current_time_ms

            if len(point.positions) != len(self.servo_ids):
                self.get_logger().error(
                    f'Expected {len(self.servo_ids)} joint positions, '
                    f'got {len(point.positions)}. Skipping point.'
                )
                continue
            
            # Check if command is for arm or gripper
            servo_positions = [
                self.angle_to_raw(position, i)
                for i, position in enumerate(reversed(point.positions))
            ]
            
            # Joint1-base -> JOint6 gripper
            # Rough method to invert joint6
            # servo_positions[0] = 1000 - servo_positions[0]
            commands = list(zip(self.servo_ids, servo_positions))

            self.get_logger().info(f'Commands:{commands}')

            # Update the virtual arm.
            virtual_msg = Int32MultiArray()
            virtual_msg.data = servo_positions

            self.virtual_positions_pub.publish(virtual_msg)

            # Publish to arm, if present.
            if self.arm is not None:
                try:
                    self.arm.setPosition(commands, wait=True, duration=duration)

                except Exception as exc:
                    self.get_logger().error( f'Arm execution failed: {exc}')
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
