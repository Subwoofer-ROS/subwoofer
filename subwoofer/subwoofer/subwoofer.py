import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import time
from adafruit_pca9685 import PCA9685
from adafruit_pca9685 import PWMChannel
import board
from busio import I2C

from std_msgs.msg import Float32
from subwoofer_interfaces.msg import ServoAngles
from sensor_msgs.msg import JointState
from subwoofer_interfaces.srv import ServoMotion
from geometry_msgs.msg import Quaternion

from tf2_ros import TransformBroadcaster, TransformStamped



class Servo:
    def __init__(self,
                 pwm_channel: int,
                 initial_angle: float = 0.0,
                 flipped: bool = False,
                 degrees_per_second: float = 1.0,
                 simulated: bool = True):
        
        self.pwm_channel = pwm_channel
        self.angle = initial_angle
        self.flipped = flipped
        self.simulated = simulated

        self.duty_cycle_range = (2_000, 10_000)
        self.angle_range = (-90.0, 90.0)

        self.aim_angle = self.angle

        if not simulated:
            self.pca = PCA9685(I2C(board.SCL, board.SDA))
            self.pca.frequency = 60
            self.channel = PWMChannel(self.pca, pwm_channel)

        self.last_servo_update = time.time()
        self.speed = degrees_per_second


    def _angle_to_dc(self, angle: float) -> int:
        offset = angle - self.angle_range[0]
        dcpd = (self.duty_cycle_range[1] - self.duty_cycle_range[0]) / (self.angle_range[1] - self.angle_range[0])
        return int(offset * dcpd + self.duty_cycle_range[0])

    def write_servo(self, angle: float) -> None:
        self.last_servo_update = time.time()

        duty_cycle = self._angle_to_dc(angle)
        if not self.simulated:
            self.channel.duty_cycle = duty_cycle
        self.angle = angle

    def update_servo(self):
        delta_time = time.time() - self.last_servo_update
        
        #speed = self.degrees_per_second
        delta_angle = self.aim_angle - self.angle
        if abs(delta_angle) <= delta_time * self.speed:
            next_angle = self.aim_angle
        else:
            offset = delta_time * self.speed
            if delta_angle < 0:
                offset *= -1
            next_angle = self.angle + offset
        self.write_servo(next_angle)

    def stop(self) -> None:
        if not self.simulated:
            self.channel.duty_cycle = 0

    def set_target(self, angle: float, speed: float) -> None:
        if angle < self.angle_range[0]:
            self.aim_angle = self.angle_range[0]
        elif angle > self.angle_range[1]:
            self.aim_angle = self.angle_range[1]
        else:
            self.aim_angle = angle
        self.speed = speed
        self.last_servo_update = time.time()


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class Subwoofer(Node):
    def __init__(self):
        super().__init__("subwoofer")
        self.declare_parameter("simulated", True)
        self.declare_parameter("update_frequency", 20 * 1e-3)

        use_sim = self.get_parameter("simulated").value
        update_frequency = self.get_parameter("update_frequency").value

        self.joint_names = [
            "front_left_hip_servo_to_front_outer_shoulder",
            "front_left_midlimb_servo_to_front_left_hip",
            "front_left_wrist_servo_to_front_left_midlimb",

            "front_right_hip_servo_to_front_outer_shoulder",
            "front_right_midlimb_servo_to_front_right_hip",
            "front_right_wrist_servo_to_front_right_midlimb",

            "back_left_hip_servo_to_back_inner_shoulder",
            "back_left_midlimb_servo_to_back_left_hip",
            "back_left_wrist_servo_to_back_left_midlimb",

            "back_right_hip_servo_to_back_inner_shoulder",
            "back_right_midlimb_servo_to_back_right_hip",
            "back_right_wrist_servo_to_back_right_midlimb",
        ]

        self.servos = [
            [
                # Leg 1 hip, upper, lower
                Servo(5, simulated=use_sim),
                Servo(2, simulated=use_sim),
                Servo(0, simulated=use_sim),
            ],
            [
                # Leg 2 hip, upper, lower
                Servo(4, simulated=use_sim),
                Servo(3, simulated=use_sim),
                Servo(1, simulated=use_sim),
            ],
            [
                # Leg 3 hip, upper, lower
                Servo(7, simulated=use_sim),
                Servo(13, simulated=use_sim),
                Servo(15, simulated=use_sim),
            ],
            [
                # Leg 4 hip, upper, lower
                Servo(6, simulated=use_sim),
                Servo(12, simulated=use_sim),
                Servo(14, simulated=use_sim),
            ],
        ]

        qos_profile = QoSProfile(depth=10)

        self.pub_joint_states = self.create_publisher(JointState,
                                               "joint_states",
                                               10)
        self.servo_timer = self.create_timer(update_frequency,
                                             self.update_servos)
        self.servo_control = self.create_service(ServoMotion,
                                                 "subwoofer/servos/update",
                                                 self.on_servo_request)
        
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)


        

    def shutdown(self):
        for servo in self.servos.values():
            servo.stop()

    def update_servos(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions = []

        for leg in self.servos:
            for servo in leg:
                servo.update_servo()
                positions.append(float(servo.angle / 180 * np.pi))
        msg.position = positions
        
        self.pub_joint_states.publish(msg)


    def on_servo_request(self, request, response):
        if request.leg < 0 or request.leg > 3:
            response.is_valid = False
            return response
        if request.servo_id < 0 or request.servo_id > 3:
            response.is_valid = False
            return response

        servo: Servo = self.servos[request.leg][request.servo_id]
        if request.velocity == 0:
            servo.stop()
        else:
            servo.set_target(request.angle, request.velocity)
            self.get_logger().info(f"Setting servo {request.leg}-{request.servo_id} to {request.angle}")
        response.is_valid = True
        
        return response





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
        rclpy.spin_once(node)
        node.destroy_node()
