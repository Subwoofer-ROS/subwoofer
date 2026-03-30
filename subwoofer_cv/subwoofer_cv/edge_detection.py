"""
Handles all code related to face detection and image processing.

This code is recommended to be offloaded onto a separate device for improved processing.
"""

# Message types
from sensor_msgs.msg import CompressedImage

# ROS libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# System libraries
import cv2
import numpy as np

class EdgeDetector(Node):
    """
    Detects motion in a video stream.
    """

    def __init__(self):
        super().__init__("motion_detection")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.detect_edges,
            10
        )

        self.edge_pub = self.create_publisher(
            CompressedImage,
            "/subwoofer/edges/compressed",
            10
        )

        self.overlay_pub = self.create_publisher(
            CompressedImage,
            "/subwoofer/edge_overlay/compressed",
            10
        )

        self.colour = (0, 0, 255)

        self.colour_change_timer = self.create_timer(
            8,
            self.colour_change
        )

        self.get_logger().info(f"Edge detector online!")

    def colour_change(self):
        self.colour = np.random.randint(0, 256, 3)
        

    def detect_edges(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)

        grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_img = cv2.GaussianBlur(grey_img, (3, 3), 0)
        sigma = np.std(blur_img)
        mean = np.mean(blur_img)
        lower = int(max(0, mean - sigma))
        upper = int(min(255, mean + sigma))
        edge_img = cv2.Canny(blur_img, lower, upper)

        response = self.bridge.cv2_to_compressed_imgmsg(edge_img)
        self.edge_pub.publish(response)


        # Overlay on 
        kernel = np.ones((2, 2), np.uint8)
        dilated_img = cv2.dilate(edge_img, kernel, iterations=1)
        _, edges = cv2.threshold(dilated_img, 0, 255, cv2.THRESH_BINARY_INV)

        edge_overlay = cv2.cvtColor(grey_img, cv2.COLOR_GRAY2BGR)
        edge_overlay[edges==0] = self.colour
        response = self.bridge.cv2_to_compressed_imgmsg(edge_overlay)
        self.overlay_pub.publish(response)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()