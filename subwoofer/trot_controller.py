# System imports
import numpy as np
import time
from enum import Enum

# ROS imports
import rclpy
from rclpy.node import Node

# Interfaces
from sensor_msgs.msg import JointState

# IK import
from subwoofer.kinematics.leg_kinematics import LegKinematics


class LegTarget:
    x: float
    y: float
    z: float


class SubwooferController(Node):
    def __init__(self):
        super().__init__("trot_controller")

        self.start_time = time.time()

        self.declare_parameter("step_length", 0.08)
        self.declare_parameter("step_height", 0.03)
        self.declare_parameter("stance_height", 0.18)
        self.declare_parameter("gait_frequency", 1.0) #Hz

        self.kinematics = LegKinematics()
        
        self.pub = self.create_publisher(
            JointState,
            "/joint_command",
            10
        )

        self.timer = self.create_timer(
            0.0025,
            self.publish_pose
        )



    def foot_target(self, t, x_offset = 0.0):
        STEP_HEIGHT = self.get_parameter("step_height").get_parameter_value().double_value
        STEP_LENGTH = self.get_parameter("step_length").get_parameter_value().double_value
        STANCE_HEIGHT = self.get_parameter("stance_height").get_parameter_value().double_value

        phase = (t % (2 * np.pi)) / (2 * np.pi)

        if phase < 0.5:
            s = phase * 2

            x = -STEP_LENGTH + 2 * STEP_LENGTH * s
            z = -STANCE_HEIGHT + STEP_HEIGHT * np.sin(np.pi * s)
        else:
            s = (phase - 0.5) * 2

            x = STEP_LENGTH - 2 * STEP_LENGTH * s
            z = -STANCE_HEIGHT

        y = 0.0

        x += x_offset

        return x, y, z


    def publish_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

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



        freq = self.get_parameter("gait_frequency").get_parameter_value().double_value
        elapsed_time = time.time() - self.start_time
        phase = 2 * np.pi * freq * elapsed_time

        # Calculate foot positions
        lf_x, lf_y, lf_z = self.foot_target(phase,         x_offset=+0.02)
        rf_x, rf_y, rf_z = self.foot_target(phase + np.pi, x_offset=+0.02)
        lh_x, lh_y, lh_z = self.foot_target(phase + np.pi, x_offset=-0.02)
        rh_x, rh_y, rh_z = self.foot_target(phase,         x_offset=-0.02)


        # Solve inverse kinematics
        lf_ik = self.kinematics.leg_ik(lf_x, lf_y, lf_z)
        if lf_ik is None:
            self.get_logger().warn(f"Unable to send inverse kinematics values.")
            return
        rf_ik = self.kinematics.leg_ik(rf_x, rf_y, rf_z)
        if rf_ik is None:
            self.get_logger().warn(f"Unable to send inverse kinematics values.")
            return
        lh_ik = self.kinematics.leg_ik(lh_x, lh_y, lh_z)
        if lh_ik is None:
            self.get_logger().warn(f"Unable to send inverse kinematics values.")
            return
        rh_ik = self.kinematics.leg_ik(rh_x, rh_y, rh_z)
        if rh_ik is None:
            self.get_logger().warn(f"Unable to send inverse kinematics values.")
            return
        

        msg.position = []




        #lf
        hip, upper, lower = lf_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)
        if abs(upper) > 1.3:
            self.get_logger().warn(
                f"Upper lf approaching limit: {upper:.2f}"
            )

        if abs(lower) > 2.0:
            self.get_logger().warn(
                f"Lower lf approaching limit: {lower:.2f}"
            )

        #rf
        hip, upper, lower = rf_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)
        if abs(upper) > 1.3:
            self.get_logger().warn(
                f"Upper rf approaching limit: {upper:.2f}"
            )

        if abs(lower) > 2.0:
            self.get_logger().warn(
                f"Lower rf approaching limit: {lower:.2f}"
            )

        #lh
        hip, upper, lower = lh_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)
        if abs(upper) > 1.3:
            self.get_logger().warn(
                f"Upper lh approaching limit: {upper:.2f}"
            )

        if abs(lower) > 2.0:
            self.get_logger().warn(
                f"Lower lh approaching limit: {lower:.2f}"
            )

        #rh
        hip, upper, lower = rh_ik
        hip += 0
        upper += 1.843
        lower -= 0.314
        msg.position.append(hip)
        msg.position.append(upper)
        msg.position.append(lower)
        if abs(upper) > 1.3:
            self.get_logger().warn(
                f"Upper rh approaching limit: {upper:.2f}"
            )

        if abs(lower) > 2.0:
            self.get_logger().warn(
                f"Lower rh approaching limit: {lower:.2f}"
            )



        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = SubwooferController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
