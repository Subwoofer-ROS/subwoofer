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

class MotionDetector(Node):
    """
    Detects motion in a video stream.
    """

    def __init__(self):
        super().__init__("motion_detection")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.detect_face,
            10
        )

        self.image_pub = self.create_publisher(
            CompressedImage,
            "/subwoofer/motion/compressed",
            10
        )

        self.max_images = 30
        self.threshold = 50
        self.blur = 7
        self.min_area = 30

        self.last_images = []

        self.get_logger().info(f"Motion detector online!")

    def append_image(self, image):
        # normalize stored images to 3-channel BGR so history is consistent
        if image is None:
            return
        if image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 1):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        self.last_images.append(image)
        if len(self.last_images) > self.max_images:
            self.last_images = self.last_images[-1*self.max_images:]

    def generate_diff(self, image: np.ndarray) -> np.ndarray:
        """
        Produce a grayscale mask (uint8: 0-255) where pixel intensity encodes how recently
        that pixel changed. More recent changes are brighter.

        - image: current BGR or gray cv2 image
        - thresh: intensity difference threshold
        - blur: gaussian blur kernel size (must be odd)
        - min_area: minimum contour area to keep (filters noise)
        """
        if image is None:
            return None

        # ensure odd blur kernel
        if self.blur % 2 == 0:
            self.blur += 1

        # get gray version of current image (handle single-channel or 3-channel inputs)
        if image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 1):
            gray = image.copy() if image.ndim == 2 else image[:, :, 0].copy()
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur, self.blur), 0)

        if not self.last_images:
            # no history yet — store and return empty mask
            self.append_image(image.copy())
            return np.zeros_like(gray, dtype=np.uint8)

        # accumulate weighted presence where more recent frames contribute more
        N = len(self.last_images)
        max_sum = N * (N + 1) // 2
        accum = np.zeros_like(gray, dtype=np.uint16)

        for idx, prev in enumerate(self.last_images):
            weight = idx + 1
            prev_gray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
            prev_gray = cv2.GaussianBlur(prev_gray, (self.blur, self.blur), 0)
            diff = cv2.absdiff(gray, prev_gray)
            _, th = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
            th_bin = (th > 0).astype(np.uint16)
            accum += th_bin * weight

        # normalize accum to 0-255 (float then back to uint8)
        scaled = (accum.astype(np.float32) / float(max_sum) * 255.0).astype(np.uint8)

        # make a binary mask from accum for contour filtering (keep any changed pixels)
        bin_mask = (accum > 0).astype(np.uint8) * 255

        # morphological cleanup on binary mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # remove small blobs using contours, but preserve brightness inside kept blobs
        contours, _ = cv2.findContours(bin_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cleaned = np.zeros_like(scaled)
        for c in contours:
            if cv2.contourArea(c) >= self.min_area:
                # create mask for this contour and copy corresponding brightness
                mask = np.zeros_like(bin_mask)
                cv2.drawContours(mask, [c], -1, 255, thickness=cv2.FILLED)
                cleaned[mask == 255] = scaled[mask == 255]

        # update history and return graded bitmap (0-255)
        self.append_image(image.copy())
        return cleaned

    def detect_face(self, msg: CompressedImage):
        """Callback for the image subscriber."""
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # keep existing behavior of storing incoming image
        self.append_image(cv_image)
        diff = self.generate_diff(cv_image)
        # convert single-channel mask to BGR for publishing on the same topic
        if diff is not None and (diff.ndim == 2 or (diff.ndim == 3 and diff.shape[2] == 1)):
            diff_bgr = cv2.cvtColor(diff, cv2.COLOR_GRAY2BGR)
        else:
            diff_bgr = diff
        self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(diff_bgr))


def main(args=None):
    rclpy.init(args=args)
    node = MotionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()