import numpy as np
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Joy
from rcl_interfaces.msg import ParameterDescriptor

from subwoofer.kinematics.leg_kinematics import LegKinematics


class StanceController(Node):
    def __init__(self):
        super().__init__("stance_controller")

        self.kinematics = LegKinematics()

        # Parameters
        self.target_height = 0.18
        self.target_pitch = 0.0
        self.target_roll = 0.0

        self.body_height = 0.18
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0

        self.roll_mult = 1.5
        self.filter_tau = 0.3

        self.last_update = time.monotonic()

        
        self.pub = self.create_publisher(
            JointState,
            "/joint_command",
            10
        )
        self.timer = self.create_timer(
            0.02,
            self.timer_callback,
        )

        self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )


    def timer_callback(self):
        # Update body values
        now = time.monotonic()
        dt = min(now - self.last_update, 0.1)
        self.last_update = now

        alpha = 1.0 - np.exp(-dt / self.filter_tau)
        self.body_height += alpha * (self.target_height - self.body_height)
        self.body_pitch += alpha * (self.target_pitch - self.body_pitch)
        self.body_roll += alpha * (self.target_roll - self.body_roll)



        msg = JointState()

        msg.name = [
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
            "back_right_wrist_servo_to_back_right_midlimb"
        ]

        # Standing pose
        lf = [ 0.02, 
               self.roll_mult*self.body_roll, 
               -self.body_height+self.body_pitch+self.body_roll]
        rf = [ 0.02, 
               -self.roll_mult*self.body_roll, 
               -self.body_height+self.body_pitch-self.body_roll]
        lh = [-0.02, 
               self.roll_mult*self.body_roll, 
               -self.body_height-self.body_pitch+self.body_roll]
        rh = [-0.02, 
               -self.roll_mult*self.body_roll, 
               -self.body_height-self.body_pitch-self.body_roll]

        # Apply IK
        lf_ik = self.kinematics.leg_ik(*lf)
        rf_ik = self.kinematics.leg_ik(*rf)
        lh_ik = self.kinematics.leg_ik(*lh)
        rh_ik = self.kinematics.leg_ik(*rh)



        # Apply offsets and add to message
        # LF
        hip, upper, lower = lf_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)

        # RF
        hip, upper, lower = rf_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)

        # LH
        hip, upper, lower = lh_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)

        # RH
        hip, upper, lower = rh_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)

        # Send message
        self.pub.publish(msg)



    def deadband(self, value: float, threshold: float = 0.05) -> float:
        if abs(value) < threshold:
            return 0.0
        return np.sign(value) * (
            (abs(value) - threshold) / 
            (1.0 - threshold)
        )

    def joy_callback(self, msg: Joy):
        self.get_logger().info(f"Received joy message with {msg.axes}")

        height_axis = self.deadband(msg.axes[1])
        pitch_axis = self.deadband(msg.axes[3])
        roll_axis = self.deadband(msg.axes[2])

        self.target_height = np.clip(
            0.14 + 0.04 * height_axis,
            0.10,
            0.18)
        self.target_pitch = np.clip(
            0.03 * pitch_axis,
            -0.03,
            0.03)
        self.target_roll = np.clip(
            0.03 * roll_axis,
            -0.03,
            0.03)



def main(args=None):
    rclpy.init(args=args)
    node = StanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()