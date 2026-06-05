import numpy as np
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String



from subwoofer.kinematics.leg_kinematics import LegKinematics
from subwoofer.robot_state import RobotState



class InterpolationLogic(str, Enum):
    LINEAR = "LINEAR"
    SINE = "SINE"



class PoseTransition:
    start_pose: list[float]
    end_pose: list[float]
    end_state: RobotState
    duration: float
    start_time: float
    interpolation: InterpolationLogic

    def __init__(self,
                 start_pose,
                 end_pose,
                 end_state,
                 duration,
                 start_time,
                 interpolation = InterpolationLogic.LINEAR):
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.end_state = end_state
        self.duration = duration
        self.start_time = start_time
        self.interpolation = interpolation


class StanceController(Node):
    def __init__(self):
        super().__init__("stance_controller")

        self.declare_parameter("height_neutral", 0.14)
        self.declare_parameter("height_max_offset", 0.04)
        self.declare_parameter("pitch_neutral", 0.0)
        self.declare_parameter("pitch_max_offset", 0.03)
        self.declare_parameter("roll_neutral", 0.0)
        self.declare_parameter("roll_max_offset", 0.03)
        self.declare_parameter("roll_shoulders", 1.5)
        self.declare_parameter("front_offset", 0.02)
        self.declare_parameter("hind_offset", -0.02)

        # Robot state
        self.robot_state = RobotState.STORAGE
        self.robot_state_sub = self.create_subscription(
            String,
            "/robot_state",
            self.robot_state_callback,
            10
        )
        self.robot_state_requester = self.create_publisher(
            String,
            "/robot_state_request",
            10
        )



        self.known_poses = {
            "trot": [
                0.0, -0.847, 1.079,
                0.0,  0.114, 0.854,
                0.0, -0.204, 1.079,
                0.0, -0.900, 0.855
            ],
            "storage": [
                0.0, -0.9, 2.5,
                0.0, -0.9, 2.5,
                0.0, -0.9, 2.5,
                0.0, -0.9, 2.5,
            ]
        }



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
            "back_right_wrist_servo_to_back_right_midlimb"
        ]


        self.kinematics = LegKinematics()

        self.trot_transition_started = False
        self.last_position_sent = None

        # Parameters
        self.target_height = self.get_parameter("height_neutral").get_parameter_value().double_value
        self.target_pitch = self.get_parameter("pitch_neutral").get_parameter_value().double_value
        self.target_roll = self.get_parameter("roll_neutral").get_parameter_value().double_value

        self.body_height = self.target_height
        self.body_pitch = self.target_pitch
        self.body_roll = self.target_roll

        self.active_transition: PoseTransition | None = None

        self.roll_mult = 1.5
        self.filter_tau = 0.3

        self.last_update = time.monotonic()


        self.current_target = None
        self.pub_stance_target = self.create_publisher(
            JointState,
            "/stance_target",
            10
        )        
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

        self.last_joint_state = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self.record_last_joint_state,
            10
        )

    def record_last_joint_state(self, msg: JointState):
        self.last_joint_state = msg.position


    def interpolate_pose(
            self,
            transition: PoseTransition
            ) -> tuple[list[float], bool]:
        
        dt = time.monotonic() - transition.start_time
        alpha = min(dt/transition.duration, 1.0)
        if transition.interpolation == InterpolationLogic.SINE:
            alpha = 0.5 - 0.5 * np.cos(np.pi * alpha)

        pose = []

        for start, end in zip(transition.start_pose, transition.end_pose):
            pose.append(
                start + alpha * (end-start)
            )
        complete = alpha >= 1.0

        return pose, complete


        
    def robot_state_callback(self, msg: String):
        try:
            new_state = RobotState(msg.data)
            if (new_state == RobotState.STANCE) ^ (self.robot_state == RobotState.STANCE):
                self.get_logger().info(f"Stance controller now {'ACTIVE' if new_state == RobotState.STANCE else 'IDLE'}.")
            self.robot_state = new_state
        except ValueError:
            self.get_logger().warn(f"Invalid state {msg.data} published.")



    def behaviour_storage(self):
        if self.robot_state != RobotState.STORAGE:
            return
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.known_poses["storage"]
        self.pub.publish(msg)

    def behaviour_stance(self):
        # Update body values
        now = time.monotonic()
        dt = min(now - self.last_update, 0.1)
        self.last_update = now

        alpha = 1.0 - np.exp(-dt / self.filter_tau)
        self.body_height += alpha * (self.target_height - self.body_height)
        self.body_pitch += alpha * (self.target_pitch - self.body_pitch)
        self.body_roll += alpha * (self.target_roll - self.body_roll)



        msg = JointState()

        msg.name = self.joint_names

        # Standing pose
        lf = [ self.get_parameter("front_offset").get_parameter_value().double_value, 
               self.roll_mult*self.body_roll, 
              -self.body_height+self.body_pitch+self.body_roll]
        rf = [ self.get_parameter("front_offset").get_parameter_value().double_value, 
              -self.roll_mult*self.body_roll, 
              -self.body_height+self.body_pitch-self.body_roll]
        lh = [ self.get_parameter("hind_offset").get_parameter_value().double_value, 
               self.roll_mult*self.body_roll, 
              -self.body_height-self.body_pitch+self.body_roll]
        rh = [ self.get_parameter("hind_offset").get_parameter_value().double_value, 
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
        self.current_target = msg.position
        self.pub_stance_target.publish(msg)
        if self.robot_state == RobotState.STANCE:
            self.last_position_sent = msg.position
            self.pub.publish(msg)



    def setup_stance_to_trot_transition(self):
        if self.last_position_sent is None:
            self.get_logger().warn(f"Trying to transition to trotting without an established stance, reverting...")
            msg = String()
            msg.data = RobotState.STANCE.value
            self.robot_state_requester.publish(msg)
            return
        
        if self.active_transition is None:
            self.active_transition = PoseTransition(
                start_pose=self.last_position_sent,
                end_pose=self.known_poses["trot"],
                start_time=time.monotonic(),
                end_state=RobotState.TROT,
                duration=0.5
            )
    
    def setup_trot_to_stance_transition(self):
        if self.last_joint_state is None:
            self.get_logger().warn(f"Trying to transition from trotting without an established stance, reverting...")
            msg = String()
            msg.data = RobotState.TROT.value
            self.robot_state_requester.publish(msg)
            return

        if self.active_transition is None:
            self.active_transition = PoseTransition(
                start_pose=self.last_joint_state,
                end_pose=self.current_target,
                start_time=time.monotonic(),
                end_state=RobotState.STANCE,
                duration=0.5
            )

    def setup_storage_to_stance_transition(self):
        if self.current_target is None:
            self.get_logger().warn(f"Trying to stand up without a target stance, reverting...")
            msg = String()
            msg.data = RobotState.STORAGE.value
            self.robot_state_requester.publish(msg)
            return
        
        if self.active_transition is None:
            self.active_transition = PoseTransition(
                start_pose=self.known_poses["storage"],
                end_pose=self.current_target,
                start_time=time.monotonic(),
                end_state=RobotState.STANCE,
                duration=2.0,
                interpolation=InterpolationLogic.SINE
            )

    def setup_stance_to_storage_transition(self):
        if self.active_transition is None:
            self.active_transition = PoseTransition(
                start_pose=self.current_target,
                end_pose=self.known_poses["storage"],
                start_time=time.monotonic(),
                end_state=RobotState.STORAGE,
                duration=2.0,
                interpolation=InterpolationLogic.SINE
            )


        
    def behaviour_transition(self):
        if self.active_transition is None:
            return
        
        # Double check that we are in a transition state
        valid_states = [
            RobotState.TRANSITION_FROM_TROT,
            RobotState.TRANSITION_TO_TROT,
            RobotState.STANDING_UP,
            RobotState.SITTING_DOWN,
        ]
        if self.robot_state not in valid_states:
            return
        
        pose, complete = self.interpolate_pose(self.active_transition)

        msg = JointState()
        msg.name = self.joint_names
        msg.position = pose
        self.pub.publish(msg)

        if complete:
            msg = String()
            msg.data = self.active_transition.end_state.value
            self.robot_state_requester.publish(msg)





        




    def timer_callback(self):
        self.behaviour_stance()

        match self.robot_state:
            case RobotState.STANCE:
                if self.active_transition is not None:
                    self.active_transition = None
            case RobotState.TRANSITION_TO_TROT:
                self.setup_stance_to_trot_transition()
                self.behaviour_transition()
            case RobotState.TRANSITION_FROM_TROT:
                self.setup_trot_to_stance_transition()
                self.behaviour_transition()
            case RobotState.STANDING_UP:
                self.setup_storage_to_stance_transition()
                self.behaviour_transition()
            case RobotState.SITTING_DOWN:
                self.setup_stance_to_storage_transition()
                self.behaviour_transition()
            case RobotState.STORAGE:
                if self.active_transition is not None:
                    self.active_transition = None
                self.behaviour_storage()

        # Clear transition state if not in a transition
        if self.active_transition is not None:
            if self.robot_state not in [
                    RobotState.TRANSITION_FROM_TROT,
                    RobotState.TRANSITION_TO_TROT,
                    RobotState.STANDING_UP,
                    RobotState.SITTING_DOWN]:
                self.active_transition = None

        



    def deadband(self, value: float, threshold: float = 0.05) -> float:
        if abs(value) < threshold:
            return 0.0
        return np.sign(value) * (
            (abs(value) - threshold) / 
            (1.0 - threshold)
        )

    def joy_callback(self, msg: Joy):
        height_axis = self.deadband(msg.axes[1])
        pitch_axis = self.deadband(msg.axes[3])
        roll_axis = self.deadband(msg.axes[2])
        neutrals = [
            self.get_parameter("height_neutral").get_parameter_value().double_value,
            self.get_parameter("pitch_neutral").get_parameter_value().double_value,
            self.get_parameter("roll_neutral").get_parameter_value().double_value
        ]
        offsets = [
            self.get_parameter("height_max_offset").get_parameter_value().double_value,
            self.get_parameter("pitch_max_offset").get_parameter_value().double_value,
            self.get_parameter("roll_max_offset").get_parameter_value().double_value
        ]

        self.target_height = np.clip(
            neutrals[0] + offsets[0] * height_axis,
            neutrals[0] - offsets[0],
            neutrals[0] + offsets[0])
        self.target_pitch = np.clip(
            neutrals[1] + offsets[1] * pitch_axis,
            neutrals[1] - offsets[1],
            neutrals[1] + offsets[1])
        self.target_roll = np.clip(
            neutrals[2] + offsets[2] * roll_axis,
            neutrals[2] - offsets[2],
            neutrals[2] + offsets[2])



def main(args=None):
    rclpy.init(args=args)
    node = StanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()