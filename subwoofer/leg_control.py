import rclpy
from rclpy.node import Node

from subwoofer_interfaces.srv import ServoMotion

class LegControl(Node):
    def __init__(self):
        super().__init__("leg")

        self.hip_client = self.create_client(ServoMotion,
                                                f"{self.get_name()}/hip/move")
        
        self.upper_client = self.create_client(ServoMotion,
                                                   f"{self.get_name()}/upper/move")
        
        self.lower_client = self.create_client(ServoMotion,
                                                   f"{self.get_name()}/lower/move")
        
        # Test value
        self.last_angle = -90.0
        self.timer = self.create_timer(5.0, self.time_cb)



    def send_angle(self, leg_index: int, angle: float, vel: float):
        match leg_index:
            case 0:
                service = self.hip_client
            case 1:
                service = self.upper_client
            case 2:
                service = self.lower_client
            case _:
                return

        msg = ServoMotion.Request()
        msg.angle = angle
        msg.velocity = vel

        self.get_logger().info(f"Sent {leg_index} move request to {msg.angle}.")
        service.call_async(msg)

    def time_cb(self):
        self.last_angle *= -1
        self.send_angle(0, self.last_angle, 90.0)



def main(args=None):
    rclpy.init(args=args)
    node = LegControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()