#!/usr/bin/env python3
"""
TtbNav – Navigation state machine
---------------------------------
Listens to /turtlebot_state (String) and converts high-level state words into
Nav2 NavigateToPose goals:

  "StartReturn"     →  origin_pos
  "StartDelivery1"  →  ws1_pos
  "StartDelivery2"  →  ws2_pos
  "Idle", "AtLoad"  →  no action

Goal points (x, y, yaw_deg) are read from parameters under the namespace
`ttb_pos_point.*`  – typically supplied via YAML.

When a new goal is accepted the node ignores further identical requests until
that goal finishes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class TtbNav(Node):
    def __init__(self):
        super().__init__('ttb_nav')

        # ------------------------------------------------------------------
        # Parameter declarations (override with YAML or launch file)
        # Each list is [x, y, yaw_deg]
        # ------------------------------------------------------------------
        self.declare_parameter('ttb_pos_point.origin_pos', [1.0, 1.0, 0.0])
        self.declare_parameter('ttb_pos_point.ws1_pos',    [3.0, 4.0, 90.0])
        self.declare_parameter('ttb_pos_point.ws2_pos',    [5.0, 6.0, 180.0])

        # Load parameters from ttb_pos_point.yaml
        self.origin_pos = self.get_parameter('ttb_pos_point.origin_pos').value
        self.ws1_pos    = self.get_parameter('ttb_pos_point.ws1_pos').value
        self.ws2_pos    = self.get_parameter('ttb_pos_point.ws2_pos').value

        self.get_logger().info(f'Origin   : {self.origin_pos}')
        self.get_logger().info(f'WS1 goal : {self.ws1_pos}')
        self.get_logger().info(f'WS2 goal : {self.ws2_pos}')

        # ------------------------------------------------------------------
        # Nav2 Action client
        # ------------------------------------------------------------------
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server…')
        self.client.wait_for_server()
        self.get_logger().info('Nav2 ready.')

        # ------------------------------------------------------------------
        # Subscribe to state commands
        # ------------------------------------------------------------------
        self.create_subscription(String, '/turtlebot_state',
                                 self.state_cb, 10)

        # Internal state to avoid multiple goals being sent repeatedly
        self.current_goal_tag = None   # remembers "origin" / "ws1" / "ws2"

    # ======================================================================
    #  Callbacks
    # ======================================================================
    def state_cb(self, msg: String):
        """Handle new state strings from GUI / supervisor."""
        state = msg.data.strip()
        self.get_logger().info(f'Received state: {state}')

        if state in ('Idle', 'AtLoad'):
            return

        # Decide which goal to send based on the received state message.
        if state in ('StartReturn', 'Origin'):
            if self.current_goal_tag != 'origin':
                self.send_goal(self.origin_pos, tag='origin')
        elif state == 'StartDelivery1':
            if self.current_goal_tag != 'ws1':
                self.send_goal(self.ws1_pos, tag='ws1')
        elif state == 'StartDelivery2':
            if self.current_goal_tag != 'ws2':
                self.send_goal(self.ws2_pos, tag='ws2')

    # ======================================================================
    #  Goal handling
    # ======================================================================
    def send_goal(self, pos_xyz, tag: str):
        """Convert [x, y, yaw_deg] → PoseStamped and send to Nav2."""
        x, y, yaw_deg = pos_xyz
        yaw_rad = math.radians(yaw_deg)
        q = quaternion_from_euler(0.0, 0.0, yaw_rad)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.get_logger().info(
            f'Sending goal "{tag}" → x:{x:.2f} y:{y:.2f} yaw:{yaw_deg:.1f}°')
        self.current_goal_tag = tag

        future = self.client.send_goal_async(goal,
                                             feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_resp_cb)

    # Log navigation feedback.
    def feedback_cb(self, fb_msg):
        """Optional: log remaining distance."""
        dist = fb_msg.feedback.distance_remaining
        self.get_logger().debug(f'  remaining: {dist:.2f} m')

    # Process the goal response from the server.
    def goal_resp_cb(self, future):
        """Handle acceptance / result of the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.current_goal_tag = None
            return

        self.get_logger().info('Goal accepted, waiting for result…')
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == 4:     # SUCCEEDED
            self.get_logger().info('Navigation succeeded.')
        elif status == 5:   # CANCELED
            self.get_logger().warn('Navigation canceled.')
        else:
            self.get_logger().error(f'Navigation failed (status {status}).')
        self.current_goal_tag = None   # allow new goals

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    nav = TtbNav()
    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
