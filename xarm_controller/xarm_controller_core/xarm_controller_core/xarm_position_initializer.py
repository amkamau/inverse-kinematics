import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class XArmPositionInitializer(Node):

    def __init__(self):
        super().__init__('xarm_position_initializer')

        self.publisher = self.create_publisher(Int32MultiArray,'/xarm/virtual_positions_raw',10)

        self.home_positions = [0, 500, 500, 500, 500, 500]

        self.start_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.publish_home)

        self.get_logger().info('Publishing initial home positions for 5 seconds.')

    def publish_home(self):
        msg = Int32MultiArray()
        msg.data = self.home_positions

        self.publisher.publish(msg)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= 5.0:
            self.get_logger().info('Initialization complete. Stopping publisher.')
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = XArmPositionInitializer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()