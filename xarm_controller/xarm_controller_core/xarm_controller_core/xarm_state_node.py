
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

import xarm

import sys


class XArmStateNode(Node):
    def __init__(self):
        super().__init__('xarm_state_node')

        self.arm = None
        self.virtual_positions = None

        self.get_logger().info(f'Python: {sys.executable}')
        try:
            import hid
            self.get_logger().info(f'HID loaded from: {hid.__file__}')
        except ImportError as exc:
            self.get_logger().error(f'HID unavailable: {exc}')

        self.positions_pub = self.create_publisher( Int32MultiArray, '/xarm/servo_positions_raw', 10)
        self.voltage_pub = self.create_publisher( Float32, '/xarm/battery_voltage', 10)
        self.virtual_positions_sub = self.create_subscription(Int32MultiArray, '/xarm/virtual_positions_raw', self.virtual_positions_callback, 10 )
        self.timer = self.create_timer(0.5, self.read_arm)
        self.get_logger().info('XArm state node started. Waiting for USB controller.')

    def virtual_positions_callback(self, msg):
        self.virtual_positions = msg

    def connect_arm(self):
        try:
            self.arm = xarm.Controller('USB')
            self.get_logger().info('XArm USB controller connected.')
            return True

        except Exception as exc:
            self.arm = None
            # self.get_logger().warning(f'XArm USB controller unavailable. Retrying... ({exc})')
            return False

    def publish_virtual_positions(self):
        if self.virtual_positions is not None:
            self.positions_pub.publish(self.virtual_positions)

    def read_arm(self):
        if self.arm is None:
            self.connect_arm()

        if self.arm is None:
            self.publish_virtual_positions()
            return

        try:
            positions = [
                1000 - self.arm.getPosition(i, degrees=False)
                if i <= 5
                else self.arm.getPosition(i, degrees=False)
                for i in range(1, 7)
            ]
            
            # self.get_logger().info(f'Arm positions: {positions}')

            msg = Int32MultiArray()
            msg.data = [int(p) for p in positions]

            self.positions_pub.publish(msg)

            voltage_msg = Float32()
            voltage_msg.data = float(self.arm.getBatteryVoltage())
            self.voltage_pub.publish(voltage_msg)

        except Exception as exc:
            self.get_logger().error(f'Arm read failed: {exc}')
            self.disconnect_arm()
            self.publish_virtual_positions()

    def disconnect_arm(self):
        if self.arm is not None:
            close = getattr(self.arm, 'close', None)

            if callable(close):
                try:
                    close()
                except Exception:
                    pass

        self.arm = None

    def destroy_node(self):
        self.disconnect_arm()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = XArmStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
