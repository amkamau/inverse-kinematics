
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

import xarm


class XArmStateNode(Node):
    def __init__(self):
        super().__init__('xarm_state_node')

        self.arm = xarm.Controller('USB')

        self.positions_pub = self.create_publisher(
            Int32MultiArray, '/xarm/servo_positions_raw', 10
        )
        self.voltage_pub = self.create_publisher(
            Float32, '/xarm/battery_voltage', 10
        )

        self.timer = self.create_timer(0.2, self.read_arm)

    def read_arm(self):
        try:
            positions = [
                self.arm.getPosition(i, degrees=False)
                for i in range(1, 7)
            ]

            msg = Int32MultiArray()
            msg.data = [int(p) for p in positions]
            self.positions_pub.publish(msg)

            voltage = float(self.arm.getBatteryVoltage())
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_pub.publish(voltage_msg)

        except Exception as exc:
            self.get_logger().error(f'Arm read failed: {exc}')


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
