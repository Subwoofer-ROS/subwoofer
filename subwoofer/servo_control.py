import rclpy
from rclpy.node import Node
import sys
import time

from adafruit_pca9685 import PCA9685
from adafruit_pca9685 import PWMChannel
import board
from busio import I2C

from std_msgs.msg import Float32
from subwoofer_interfaces.srv import ServoMotion


class ServoControl(Node):
    """
    The Servo node is responsible for controlling a single servo through ROS interactions.
    The node is defined with an I2C address which should be unique (no checks implemented yet.)

    When an angle is requested, this angle will be checked with limit values, and set as the
    aim angle.
    Every n seconds, will move the servo towards the aim angle at a set speed to emulate
    continuous movement.
    """

    def __init__(self):
        """Initialises the Servo node."""
        super().__init__("servo")

        self.declare_parameter("simulated", True)
        self.declare_parameter("pwm_channel", -1)
        self.declare_parameter("min_duty_cycle", 2000)
        self.declare_parameter("max_duty_cycle", 10000)
        self.declare_parameter("min_angle", -90)
        self.declare_parameter("max_angle", 90)
        self.declare_parameter("max_velocity", 90)
        self.declare_parameter("flipped", False)    
        self.declare_parameter("initial_angle", 0)

        self.current_angle = self.get_parameter("initial_angle").value
        self.aim_angle = self.current_angle
        self.get_logger().info(f"Initial angle: {self.get_parameter('initial_angle').value}")
        self.current_speed = self.get_parameter("max_velocity").value

        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value
        self.min_duty_cycle = self.get_parameter("min_duty_cycle").value
        self.max_duty_cycle = self.get_parameter("max_duty_cycle").value
        self.is_flipped = self.get_parameter("flipped").value

        self.servo_init()

        # Publishers
        self.pub_servo_angle = self.create_publisher(Float32,
                                                     f"{self.get_name()}/angle",
                                                     10)
        self.pub_servo_duty_cycle = self.create_publisher(Float32,
                                                     f"{self.get_name()}/duty_cycle",
                                                     10)

        # Timers
        self.servo_update_timer = self.create_timer(0.1,
                                                    self.update_servo)
        
        # Services
        self.servo_update_service = self.create_service(ServoMotion,
                                                        f"{self.get_name()}/move",
                                                        self.on_servo_request)

    def shutdown(self):
        self.channel.duty_cycle = 0

    def servo_init(self):
        """
        Creates and sets up the hardware functionality.
        If simulated, will skip hardware-requiring features.
        """
        self.simulated = self.get_parameter("simulated").value
        if not self.simulated:
            self.pca = PCA9685(I2C(board.SCL, board.SDA))
            self.pca.frequency = 60
            self.channel = PWMChannel(self.pca, self.get_parameter("pwm_channel").value)

        self.last_servo_update = time.time()
        self.get_logger().info(f"Servo initialised successfully!")

    def update_servo(self):
        """
        Updates the servo angle.
        Will shift the servo angle by however far it should have moved assuming linear velocity.
        """
        self.get_logger().debug(f"Updating servo location.")

        delta_time = time.time() - self.last_servo_update
        vel = self.current_speed

        delta_angle = self.aim_angle - self.current_angle
        if abs(delta_angle) <= delta_time * vel:
            new_angle = self.aim_angle
        else:
            offset = delta_time * vel
            if delta_angle < 0:
                offset *= -1
            new_angle = self.current_angle + offset

        self.write_to_servo(new_angle)
        self.last_servo_update = time.time()
        
        msg = Float32()
        msg.data = float(self.current_angle)
        self.pub_servo_angle.publish(msg)


    def on_servo_request(self, request, response):
        """
        When called, sets the aimed angle.
        Will overwrite the current aim & velocity.

        Both the supplied angle and velocity will be capped by the servo limits.
        If no velocity is supplied, the default max velocity is used.
        """
        self.get_logger().info(
            f"Received move request angle:{self.aim_angle}, vel:{request.velocity}, curr_vel:{self.current_speed}")
        self.aim_angle = max(min(request.angle, self.max_angle), self.min_angle)
        if request.velocity == 0.0:
            self.current_speed = self.get_parameter("max_velocity").value
        else:
            self.current_speed = max(min(request.velocity, self.get_parameter("max_velocity").value), 0)
        response.is_valid = True
        return response


    def write_to_servo(self, angle: float):
        """
        Writes an angle to the servo channel.
        Additionally publishes the duty cicle written.
        """
        duty_cycle = self._angle_to_duty_cycle(angle)
        if self.is_flipped:
            duty_cycle = self.max_duty_cycle - duty_cycle
        msg = Float32()
        msg.data = float(duty_cycle)
        self.pub_servo_duty_cycle.publish(msg)

        if not self.simulated:
            self.channel.duty_cycle = duty_cycle

        self.current_angle = angle

    def _angle_to_duty_cycle(self, angle: int) -> int:
        """Convert an angle to its respective duty cycle."""
        angle_offset_from_min = angle - self.min_angle
        
        duty_cycle_per_degree = (self.max_duty_cycle-self.min_duty_cycle)/(self.max_angle-self.min_angle)
        duty_cycle_offset = angle_offset_from_min * duty_cycle_per_degree
        
        return int(duty_cycle_offset + self.min_duty_cycle)
    
    def _duty_cycle_to_angle(self, duty_cycle: int) -> float|int:
        """Convert a duty cycle to its respective angle."""
        dc_offset = duty_cycle - self.min_duty_cycle
        degree_per_dc = (self.max_angle-self.min_angle)/(self.max_duty_cycle-self.min_duty_cycle)
        return dc_offset * degree_per_dc + self.min_angle



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        node.destroy_node()
