"""
Handles all code related to face detection and image processing.

This code is recommended to be offloaded onto a separate device for improved processing.
"""

# Message types
from sensor_msgs.msg import CompressedImage, Image
from subwoofer_interfaces.msg import FacePoint, FacePoints

# ROS libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# System libraries
import cv2
import os
import numpy as np

class FaceDetection(Node):
    """
    This node will look for faces and publish any found.

    The node will await an image topic on `/image_raw/compressed`.
    If found, this image will be processed for faces using the Viola-Jones face detection algorithm.
    Any faces get highlighted in green (0x00ff00).
    Each face has their location published, as we ass the highlighted image.
    """

    last_image = None
    outline_colour: tuple[int, int, int] = (0, 255, 0)

    def __init__(self):
        """TODO: Add description."""
        super().__init__("face_detection")

        self.param_cascade_path = self.declare_parameter("cascade_path", None)
        self.cascade_path = self.get_parameter("cascade_path").value
        if self.cascade_path is None:
            self.cascade_path = os.path.join(
                os.environ["SW_PATH"],
                "static",
                "harrcascade_frontalface_default.xml"
            )
        self.get_logger().info(f"Cascade path: {self.cascade_path}")
        try:
            self.cascade = cv2.CascadeClassifier(self.cascade_path)
        except:
            self.get_logger().fatal("Unable to load cascade file.")
            self.destroy_node()
            return


        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.detect_face,
            10
        )

        self.face_detect_pub = self.create_publisher(
            FacePoints,
            "/subwoofer/face_detector/points",
            10
        )

        self.face_box_compressed_pub = self.create_publisher(
            CompressedImage,
            "/subwoofer/face_detector/compressed",
            10
        )

        self.face_box_pub = self.create_publisher(
            Image,
            "/subwoofer/face_detector",
            10
        )

        self.get_logger().info(f"Face detector online!")


    def detect_face(self, msg) -> None:
        """
        Whenever an image is published, analyse it for faces and outline them.

        :param msg CompressedImage: published image.
        """
        # Read in most recent image
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.last_image = img

        # Determine where there are faces
        rects = self.cascade.detectMultiScale(
            img,
            scaleFactor=1.3,
            minNeighbors=4,
            minSize=(30,30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        # If any faces detected, draw boxes over them
        if len(rects) > 0:
            rects[:,2:] += rects[:,:2]
            msg1 = FacePoints()

            for x1, y1, x2, y2 in rects:
                r = FacePoint()
                r.x = int(abs((x2-x1)//2))
                r.y = int(abs((y2-y1)//2))
                r.width = int(abs(x2-x1))
                r.height = int(abs(y2-y1))
                msg1.points.append(r)
            
            self.face_detect_pub.publish(msg1)
            self.draw_boxes(img, rects, self.outline_colour)

        # Re-compress image, then publish it
        msg2 = self.bridge.cv2_to_compressed_imgmsg(img)
        self.face_box_compressed_pub.publish(msg2)

        # Publish non-compressed image
        msg3 = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.face_box_pub.publish(msg3)



    def draw_boxes(self, image: np.ndarray, boxes: list[int, int, int, int], colour: tuple[int, int, int]) -> None:
        """
        Draws a box on the given image at the coordinates given in the boxes variable.

        :param image np.dnarray: The opencv image
        :param boxes list[int, int, int, int]: Coordinates for the box corners
        :param colour tuple[int, int, int]: What colour the box should be draws as.
        """
        for x1, y1, x2, y2 in boxes:
            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)




def main(args=None):
    rclpy.init(args=args)
    node = FaceDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()