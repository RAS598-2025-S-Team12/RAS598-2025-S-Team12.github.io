#!/usr/bin/env python3
"""
This node processes the incoming RGB image to detect blocks of two colors:
red and blue. It then determines which color is dominant (by comparing the
area of the detected blocks) and publishes a corresponding command message:
- "StartDelivery1" if red is dominant.
- "StartDelivery2" if blue is dominant.
The published command can be used by other nodes (e.g., navigation) to decide
which designated workstation the turtlebot should go.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlockDetector(Node):
    def __init__(self):
        super().__init__('block_detector')
        self.subscription = self.create_subscription(
            Image,
            '/rpi_14/oak/rgb/image_raw',
            self.image_callback,
            10
        )
        # Publish the detected color command on topic /turtlebot_14/block_info
        self.publisher_ = self.create_publisher(String, 'turtlebot_14/block_info', 10)
        self.bridge = CvBridge()
        # timer to publish at a slower rate (can be adjusted as needed)
        self.timer = self.create_timer(0.5, self.publish_detected_color)
        # Store detected color info in a variable (latest decision)
        self.detected_command = None

    def image_callback(self, msg):
        """Process color image and detect red and blue blocks."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red thresholds (combine two ranges)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Blue thresholds
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find blocks for red and blue only.
        red_blocks = self.find_blocks(mask_red, 'red')
        blue_blocks = self.find_blocks(mask_blue, 'blue')

        # Sum block areas for each color (or pick maximum area block)
        red_area = sum([block['area'] for block in red_blocks])
        blue_area = sum([block['area'] for block in blue_blocks])

        # Decide on the dominant color based on total area.
        if red_area > blue_area and red_area > 0:
            self.detected_command = "StartDelivery1"
        elif blue_area > red_area and blue_area > 0:
            self.detected_command = "StartDelivery2"
        else:
            self.detected_command = None

        # (Optional) Draw detection result on image for debugging.
        for block in red_blocks + blue_blocks:
            x, y, w, h = block['bbox']['x'], block['bbox']['y'], block['bbox']['w'], block['bbox']['h']
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.putText(cv_image, block['color'], (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Color Detection", 640, 480)
        cv2.imshow("Color Detection", cv_image)
        cv2.waitKey(1)

    def find_blocks(self, mask, color):
        """Find contours in the mask and return a list of block info including bounding box and area."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        block_list = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:  # filter out noise
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            block_list.append({
                'color': color,
                'area': area,
                'bbox': {'x': x, 'y': y, 'w': w, 'h': h}
            })
        return block_list

    def publish_detected_color(self):
        """Publish the detected color command if available."""
        if self.detected_command is not None:
            msg = String()
            msg.data = self.detected_command
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published command: {self.detected_command}")

def main(args=None):
    rclpy.init(args=args)
    node = BlockDetector()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
