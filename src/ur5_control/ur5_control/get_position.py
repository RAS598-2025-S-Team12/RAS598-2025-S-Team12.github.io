import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class UR5CombinedListener(Node):
    def __init__(self):
        super().__init__('ur5_combined_listener')

        self.joint_positions = {}
        self.ee_position = None

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pose_callback,
            10
        )

        # Timer to periodically print combined output
        self.timer = self.create_timer(1.0, self.print_data)

    def joint_callback(self, msg: JointState):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        self.ee_position = (pos.x, pos.y, pos.z)

    def print_data(self):
        if self.ee_position:
            x, y, z = self.ee_position
            self.get_logger().info(f'EE Position -> x: {x:.4f}, y: {y:.4f}, z: {z:.4f}')
        else:
            self.get_logger().info('Waiting for EE pose...')

        if self.joint_positions:
            self.get_logger().info('Joint Angles (radians):')
            for joint, angle in self.joint_positions.items():
                self.get_logger().info(f'  {joint}: {angle:.4f}')
        else:
            self.get_logger().info('Waiting for joint states...')


def main(args=None):
    rclpy.init(args=args)
    node = UR5CombinedListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
