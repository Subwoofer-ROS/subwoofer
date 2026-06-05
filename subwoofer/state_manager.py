from enum import Enum

from rclpy.node import Node
import rclpy

from std_msgs.msg import String

from subwoofer.robot_state import RobotState


class StateManager(Node):
    def __init__(self):
        super().__init__("subwoofer_behaviour_manager")

        self.robot_state: RobotState = RobotState.STORAGE

        self.state_pub = self.create_publisher(
            String,
            "/robot_state",
            10
        )

        self.state_request = self.create_subscription(
            String,
            "/robot_state_request",
            self.state_request_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info("State Manager online!")

    def timer_callback(self):
        msg = String()
        msg.data = self.robot_state.value
        self.state_pub.publish(msg)

    def state_request_callback(self, msg: String):
        try:
            request = RobotState(msg.data)
        except ValueError:
            self.get_logger().warn(f"State manager received request for invalid state {msg.data}.")
            return
        
        if request != self.robot_state:
            self.get_logger().info(f"Changing state {self.robot_state.value} -> {request.value}")
        

        if request == self.robot_state:
            return

        match request:
            case RobotState.STANCE:
                if self.robot_state == RobotState.TROT:
                    self.robot_state = RobotState.TRANSITION_FROM_TROT
                    return
                if self.robot_state == RobotState.TRANSITION_FROM_TROT:
                    self.robot_state = RobotState.STANCE
                    return
                if self.robot_state == RobotState.STORAGE:
                    self.robot_state = RobotState.STANDING_UP
                    return
                if self.robot_state == RobotState.STANDING_UP:
                    self.robot_state = RobotState.STANCE
                    return
            case RobotState.TROT:
                if self.robot_state == RobotState.STANCE:
                    self.robot_state = RobotState.TRANSITION_TO_TROT
                    return
                if self.robot_state == RobotState.TRANSITION_TO_TROT:
                    self.robot_state = RobotState.TROT
                    return
                if self.robot_state == RobotState.TRANSITION_FROM_TROT:
                    self.robot_state = RobotState.TROT
                    return
            case RobotState.TRANSITION_TO_TROT:
                if self.robot_state == RobotState.STANCE:
                    self.robot_state = RobotState.TRANSITION_TO_TROT
                    return
            case RobotState.TRANSITION_FROM_TROT:
                if self.robot_state == RobotState.TROT:
                    self.robot_state = RobotState.TRANSITION_FROM_TROT
                    return
            case RobotState.STORAGE:
                if self.robot_state == RobotState.STANCE:
                    self.robot_state = RobotState.SITTING_DOWN
                    return
                if self.robot_state == RobotState.SITTING_DOWN:
                    self.robot_state = RobotState.STORAGE
                    return
            case RobotState.SITTING_DOWN:
                if self.robot_state == RobotState.STANCE:
                    self.robot_state = RobotState.SITTING_DOWN
                    return
            case RobotState.STANDING_UP:
                if self.robot_state == RobotState.STORAGE:
                    self.robot_state = RobotState.STANDING_UP
                    return

                
        
        self.get_logger().warn(f"Unexpected transition state {self.robot_state.value} -> {request.value}")



        


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()