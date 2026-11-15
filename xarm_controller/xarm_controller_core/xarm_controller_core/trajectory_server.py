import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class TrajectoryServer(Node):

    def __init__(self):
        super().__init__('trajectory_server')

        # Publish the raw trajectory for the processing node.
        self.arm_trajectory_pub = self.create_publisher(JointTrajectory, '/raw_trajectory_1_4', 10)
        self.wrist_trajectory_pub = self.create_publisher(JointTrajectory, '/raw_trajectory_5', 10)
        self.gripper_trajectory_pub = self.create_publisher(JointTrajectory, '/raw_trajectory_6', 10)

        # MoveIt action interface.
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.wrist_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/wrist_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            execute_callback=self.gripper_execute_callback
        )
        self.get_logger().info('Trajectory action server is ready.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received trajectory goal.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancellation request.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        trajectory = goal_handle.request.trajectory
        joint_names = set(trajectory.joint_names)

        self.get_logger().info(
            f'Received trajectory with {len(trajectory.points)} points '
            f'for joints: {trajectory.joint_names}'
        )

        # Route trajectory according to its joints.
        if joint_names == {'joint1', 'joint2', 'joint3', 'joint4'}:
            self.arm_trajectory_pub.publish(trajectory)
            topic = '/raw_trajectory_1_4'

        elif joint_names == {'joint5'}:
            self.wrist_trajectory_pub.publish(trajectory)
            topic = '/raw_trajectory_5'

        else:
            self.get_logger().error(f'Unsupported joint combination: {trajectory.joint_names}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = 'Unsupported joint combination.'

            return result

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = f'Trajectory forwarded to {topic}.'

        goal_handle.succeed()

        self.get_logger().info(f'Trajectory published to {topic}.')

        return result
    
    def gripper_execute_callback(self, goal_handle):

        position = goal_handle.request.command.position
        max_effort = goal_handle.request.command.max_effort

        self.get_logger().info(
            f'Gripper command received: position={position}, '
            f'max_effort={max_effort}'
        )

        # Convert GripperCommand into a JointTrajectory.
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint6']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 2

        trajectory.points.append(point)

        self.gripper_trajectory_pub.publish(trajectory)

        result = GripperCommand.Result()
        result.position = position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True

        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
