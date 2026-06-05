from rclpy.node import Node
import rclpy

from sensor_msgs.msg import Joy
from std_msgs.msg import String

from subwoofer.robot_state import RobotState


class RemoteController(Node):
    def __init__(self):
        super().__init__("remote_controller")

        self.robot_state = None

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.state_requester = self.create_publisher(
            String,
            "/robot_state_request",
            10
        )
        self.state_sub = self.create_subscription(
            String,
            "/robot_state",
            self.robot_state_callback,
            10
        )

        self.buttons = {
            "x": 0,
            "y": 0,
            "a": 0,
            "b": 0
        }
        self.button_max = 2


    def request_state(self, state: RobotState):
        try:
            msg = String()
            msg.data = state.value
            self.state_requester.publish(msg)
        except ValueError:
            pass

    def robot_state_callback(self, msg: String):
        self.robot_state = RobotState(msg.data)

    def button_pressed(self, btn):
        return self.buttons[btn] > 0
    
    def button_just_pressed(self, btn):
        return self.buttons[btn] == self.button_max

    def joy_callback(self, msg: Joy):
        btn_label = ["b", "a", "x", "y"]
        btn_index = [0, 1, 2, 3]

        # Process button presses
        for btn, idx in zip(btn_label, btn_index):
            status = msg.buttons[idx]
            if not status:
                self.buttons[btn] = max(self.buttons[btn]-1, 0)
                continue
            
            if not self.button_pressed(btn):
                self.buttons[btn] = self.button_max
                continue

            self.buttons[btn] = max(self.buttons[btn]-1, 1)


        # Ignore transition states
        if self.robot_state in [
            RobotState.TRANSITION_FROM_TROT,
            RobotState.TRANSITION_TO_TROT]:
            return

        # Determine behaviour
        if self.button_just_pressed("a"): # Stance
            match self.robot_state:
                case RobotState.TROT:
                    self.request_state(RobotState.TRANSITION_FROM_TROT)
                case RobotState.STORAGE:
                    self.request_state(RobotState.STANDING_UP)
                case _:
                    pass

        elif self.button_just_pressed("b"): # Trot
            match self.robot_state: 
                case RobotState.STANCE:
                    self.request_state(RobotState.TRANSITION_TO_TROT)
                case _:
                    pass

        elif self.button_just_pressed("x"): # Storage
            match self.robot_state:
                case RobotState.STANCE:
                    self.request_state(RobotState.SITTING_DOWN)
            pass






def main(args=None):
    rclpy.init(args=args)
    node = RemoteController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()