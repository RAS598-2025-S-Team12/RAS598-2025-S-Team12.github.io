import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class UR5Mover(Node):
    def __init__(self):
        super().__init__('ur5_mover')

        # Publisher to the joint trajectory controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Wait a bit for things to initialize
        self.timer = self.create_timer(2.0, self.send_command)
        self.done = False

    def send_command(self):
        if self.done:
            return

        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]


        # Each joint configuration in degrees
        sequence_degrees = [
            [-154, -41, 104, -206, -90, 3],          # A pick
            [-154, -65, 100, -233, -90, 3],     # B
            [-75, -58,  90, 237, -95, 82],    # C
            [-75, -35, 23, 210, -95, 82],       # D place
        ]

        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

#        pause_duration = 2  # seconds to pause at each position
#        move_duration = 3   # time it takes to move to each point

        for i, joint_degrees in enumerate(sequence_degrees):
            point = JointTrajectoryPoint()
            point.positions = [math.radians(deg) for deg in joint_degrees]
            point.time_from_start.sec = (i + 1) * 2  # 2 seconds per movement
            traj_msg.points.append(point)

        self.get_logger().info('Publishing pick-and-place trajectory...')
        self.publisher_.publish(traj_msg)
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = UR5Mover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
