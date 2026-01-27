import rclpy
from rclpy.node import Node
import sys

class ControlUI(Node):
    def __init__(self):
        
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()