#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray


class CmdController(Node):
    """
    Forward /gui_cmd_vel to /c3_14/cmd_vel and broadcast default velocities.
    Reacts only to 'Emergency Stop' from /turtlebot_state.
    """

    def __init__(self):
        super().__init__('cmd_controller')

        # publishers
        self.pub_cmd     = self.create_publisher(
            Twist, '/c3_14/cmd_vel', qos_profile_system_default)
        self.pub_default = self.create_publisher(
            Float32MultiArray, '/default_vel', 10)

        # subscribers
        self.create_subscription(
            Twist, '/gui_cmd_vel', self.twist_callback, 10)
        self.create_subscription(
            String, '/turtlebot_state', self.state_callback, 10)

        # default velocity
        self.default_lin = 0.3
        self.default_ang = 2.0
        self.publish_default()

        self.get_logger().info('cmd_controller node started (relay mode)')

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------
    def twist_callback(self, msg: Twist):
        """Relay GUI twist to robot."""
        self.pub_cmd.publish(msg)
        self.get_logger().debug(
            f'Relay Twist  lin={msg.linear.x:.2f}  ang={msg.angular.z:.2f}')

    def state_callback(self, msg: String):
        """Only react to Emergency Stop."""
        if msg.data == 'Emergency Stop':
            self.get_logger().warn('Emergency Stop received!')
            self.stop_robot()

    # -------------------------------------------------
    # Helpers
    # -------------------------------------------------
    def publish_default(self):
        """Publish default linear & angular velocity once on startup."""
        m = Float32MultiArray()
        m.data = [self.default_lin, self.default_ang]
        self.pub_default.publish(m)
        self.get_logger().info(
            f'Published default_vel lin={self.default_lin} ang={self.default_ang}')

    def stop_robot(self):
        """Send zero Twist a few times to guarantee stop."""
        zero = Twist()
        for _ in range(10):
            self.pub_cmd.publish(zero)
            time.sleep(0.05)
        self.get_logger().info('Robot stopped.')

# -----------------------------------------------------
# main
# -----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt – shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
