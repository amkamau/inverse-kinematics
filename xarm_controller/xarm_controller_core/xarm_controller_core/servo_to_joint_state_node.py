
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray


class ServoToJointStateNode(Node):
    def __init__(self):
        super().__init__('servo_to_joint_state')

        # Provisional calibration values for all six joints.
        self.declare_parameter('raw_min', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('raw_max', [1000, 1000, 1000, 1000, 1000, 1000])
        self.declare_parameter('angle_min',[-2.7, -1.5, -1.5, -1.5, -1.5, 0.0])
        self.declare_parameter('angle_max', [2.7, 1.5, 1.5, 1.5, 1.5, 1.5708])

        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

        self.raw_min = self.get_parameter('raw_min').value
        self.raw_max = self.get_parameter('raw_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value

        self._validate_calibration()
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription( Int32MultiArray, '/xarm/servo_positions_raw', self.servo_callback, 10)
        self.get_logger().info('Servo-to-joint-state bridge started. ' 'Publishing all six joints.')

    def _validate_calibration(self):
        arrays = [self.raw_min, self.raw_max, self.angle_min, self.angle_max]

        if any(len(values) != 6 for values in arrays):
            raise ValueError('Each calibration parameter must contain six values.')

        for i in range(6):
            if self.raw_max[i] <= self.raw_min[i]:
                raise ValueError(f'raw_max must exceed raw_min for joint{i + 1}.')

            if self.angle_max[i] <= self.angle_min[i]:
                raise ValueError(f'angle_max must exceed angle_min for joint{i + 1}.')

    def servo_callback(self, msg):
        if len(msg.data) != 6:
            self.get_logger().warning(f'Expected 6 servo values, received {len(msg.data)}.')
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names

        positions = []

        # Reverse servo order: servo 6 -> joint1, servo 5 -> joint2, etc.
        servo_indices = [5, 4, 3, 2, 1, 0]

        for i in range(6):
            raw = float(msg.data[servo_indices[i]])

            # Clamp to configured calibration range.
            raw = max(self.raw_min[i], min(self.raw_max[i], raw))

            # Convert raw servo units to radians.
            fraction = ((raw - self.raw_min[i]) / (self.raw_max[i] - self.raw_min[i]))

            angle = (self.angle_min[i] + fraction * (self.angle_max[i] - self.angle_min[i]))

            positions.append(angle)

        joint_msg.position = positions
        self.publisher.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoToJointStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
