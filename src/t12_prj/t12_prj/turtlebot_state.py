import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class TurtleBotState(Node):
    def __init__(self):
        super().__init__('turtlebot_state')
        self.pub_state = self.create_publisher(String, '/turtlebot_state', 10)
        self.sub_gui_state = self.create_subscription(String, '/turtlebot_state', self.state_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/c3_14/cmd_vel', self.vel_callback, 10)

        self.action_in_progress = None
        self.zero_cmd_time = None

        self.timer = self.create_timer(1.0, self.check_arrival_condition)
        self.get_logger().info('turtlebot_state node started')

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.pub_state.publish(msg)
        self.get_logger().info(f"State updated: {text}")

    def state_callback(self, msg):
        if "Action A" in msg.data:
            self.action_in_progress = 'A'
            self.zero_cmd_time = None
        elif "Action B" in msg.data:
            self.action_in_progress = 'B'
            self.zero_cmd_time = None
        elif "Emergency Stop" in msg.data:
            self.action_in_progress = None
            self.zero_cmd_time = None

    def vel_callback(self, msg):
        if self.action_in_progress:
            if msg.linear.x == 0.0 and msg.angular.z == 0.0:
                if self.zero_cmd_time is None:
                    self.zero_cmd_time = time.time()
            else:
                self.zero_cmd_time = None

    def check_arrival_condition(self):
        if self.action_in_progress and self.zero_cmd_time:
            if time.time() - self.zero_cmd_time >= 3.0:
                if self.action_in_progress == 'A':
                    self.publish_state("Arrived at A")
                elif self.action_in_progress == 'B':
                    self.publish_state("Arrived at B")
                self.action_in_progress = None
                self.zero_cmd_time = None

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
