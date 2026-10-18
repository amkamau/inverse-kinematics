
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import xarm


class XArmCommandNode(Node):
    def __init__(self):
        super().__init__('xarm_command_node')

        self.arm = xarm.Controller('USB')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/xarm/set_positions_raw',
            self.set_positions,
            10
        )

    def set_positions(self, msg):
        values = list(msg.data)

        if not values or len(values) % 2 != 0:
            self.get_logger().error(
                'Expected pairs: servo_id, target_position'
            )
            return

        commands = []

        for i in range(0, len(values), 2):
            servo_id, target = values[i:i + 2]

            if not 1 <= servo_id <= 6:
                self.get_logger().error(
                    f'Invalid servo ID: {servo_id}'
                )
                return

            # Conservative validation; confirm the valid target
            # range supported by your controller before use.
            if not 0 <= target <= 1000:
                self.get_logger().error(
                    f'Invalid target value: {target}'
                )
                return

            commands.append((servo_id, target))

        try:
            self.arm.setPosition(commands, wait=True)
            self.get_logger().info(f'Position command sent: {commands}')
        except Exception as exc:
            self.get_logger().error(f'Arm command failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = XArmCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
