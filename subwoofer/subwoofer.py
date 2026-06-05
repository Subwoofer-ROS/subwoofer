# System imports
import numpy as np

# Hardware interface
from adafruit_pca9685 import PCA9685
from adafruit_pca9685 import PWMChannel
import board
from busio import I2C

# ROS imports
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState



class Servo:
    def __init__(self,
                 pwm_channel: int,
                 initial_angle: float = 0.0,
                 angle_offset: float = 0.0,
                 dc_per_radian: float = 8_000/np.pi,
                 dc_center: int = 6_000,
                 mirror: bool = False,
                 simulated: bool = True):
        
        self.pwm_channel = pwm_channel
        self.simulated = simulated
        self.mirror = mirror
        self.center = dc_center
        self.dcpr = dc_per_radian
        self.angle_offset = angle_offset

        if not simulated:
            self.pca = PCA9685(I2C(board.SCL, board.SDA))
            self.pca.frequency = 60
            self.channel = PWMChannel(self.pca, pwm_channel)

        self.set_servo(initial_angle)


    def _angle_to_dc(self, angle: float) -> int:
        delta_dc = angle * self.dcpr
        if self.mirror: 
            delta_dc *= -1
        return int(np.floor(self.center + delta_dc))

    def set_servo(self, angle) -> None:
        self.angle = angle + self.angle_offset
        if not self.simulated:
            try:
                self.channel.duty_cycle =  self._angle_to_dc(self.angle)
            except OSError as e:
                print(f"Error trying to send duty cycle")

    def get_angle(self) -> float:
        return self.angle - self.angle_offset

    def stop(self) -> None:
        if not self.simulated:
            self.channel.duty_cycle = 0



class Subwoofer(Node):
    def __init__(self):
        super().__init__("subwoofer")
        self.declare_parameter("simulated", True)
        self.declare_parameter("update_frequency", 0.02)

        use_sim = self.get_parameter("simulated").get_parameter_value().bool_value
        self.get_logger().info(f"Subwoofer launching as {'SIMULATED' if use_sim else 'LIVE'}")

        self.joints = {
            "front_left_hip_servo_to_front_outer_shoulder": Servo(11,
                                                                  mirror=True,
                                                                  simulated=use_sim),
            "front_left_midlimb_servo_to_front_left_hip": Servo(13,
                                                                mirror=True,
                                                                initial_angle=-0.9,
                                                                simulated=use_sim),
            "front_left_wrist_servo_to_front_left_midlimb": Servo(15,
                                                                  mirror=True,
                                                                  initial_angle=2.5,
                                                                  angle_offset=-1.07,
                                                                  simulated=use_sim),

            "front_right_hip_servo_to_front_outer_shoulder": Servo(10,
                                                                   mirror=False,
                                                                   simulated=use_sim),
            "front_right_midlimb_servo_to_front_right_hip": Servo(12,
                                                                  mirror=False,
                                                                  initial_angle=-0.9,
                                                                  simulated=use_sim),
            "front_right_wrist_servo_to_front_right_midlimb": Servo(14,
                                                                    mirror=False,
                                                                    initial_angle=2.5,
                                                                    angle_offset=-1.07,
                                                                    simulated=use_sim),

            "back_left_hip_servo_to_back_inner_shoulder": Servo(5,
                                                                mirror=False,
                                                                simulated=use_sim),
            "back_left_midlimb_servo_to_back_left_hip": Servo(3,
                                                              mirror=True,
                                                              initial_angle=-0.9,
                                                              simulated=use_sim),
            "back_left_wrist_servo_to_back_left_midlimb": Servo(1,
                                                                mirror=True,
                                                                initial_angle=2.5,
                                                                angle_offset=-1.07,
                                                                simulated=use_sim),

            "back_right_hip_servo_to_back_inner_shoulder": Servo(4,
                                                                 mirror=True,
                                                                 simulated=use_sim),
            "back_right_midlimb_servo_to_back_right_hip": Servo(2,
                                                                mirror=False,
                                                                initial_angle=-0.9,
                                                                simulated=use_sim),
            "back_right_wrist_servo_to_back_right_midlimb": Servo(0,
                                                                  mirror=False,
                                                                  initial_angle=2.5,
                                                                  angle_offset=-1.07,
                                                                  simulated=use_sim),
        }

        self.last_position_request: dict[str, float] | None = None
        
        self.sub_joint_states = self.create_subscription(JointState,
                                                         "joint_command",
                                                         self.on_joint_states,
                                                         1)


        self.pub_servo_positions = self.create_publisher(JointState,
                                                        "joint_states",
                                                        10)
        self.servo_state_timer = self.create_timer(0.1, self.servo_states)

        self.servo_write = self.create_timer(
            self.get_parameter("update_frequency").get_parameter_value().double_value,
            self.servo_update
        )

        self.get_logger().info("Subwoofer is online")




    def shutdown(self):
        for servo in self.joints.values():
            servo.stop()



    def on_joint_states(self, msg: JointState):
        pose = {}
        for name, position in zip(msg.name, msg.position):
            if name not in self.joints.keys():
                self.get_logger().warning(f"Joint {name} is invalid, ignoring...")
                continue
            pose[name] = position
        self.last_position_request = pose

    def servo_update(self):
        if self.last_position_request is None:
            return
        for name, pose in self.last_position_request.items():
            servo = self.joints[name]
            try:
                servo.set_servo(pose)
            except OSError as e:
                self.get_logger().error(f"I2C write failed on servo {name}")


    def servo_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        names = []
        values = []
        dcs = []
        for name, servo in self.joints.items():
            names.append(name)
            values.append(servo.get_angle())
            dcs.append(servo._angle_to_dc(servo.angle))
        msg.name = names
        msg.position = values

        self.get_logger().debug(f"Joint values: {values}")
        self.get_logger().debug(f"Duty Cycles: {dcs}")

        self.pub_servo_positions.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Subwoofer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    except rclpy.executors.ExternalShutdownException:
        node.shutdown()
    finally:
        node.destroy_node()
