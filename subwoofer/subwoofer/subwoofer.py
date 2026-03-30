import rclpy
from rclpy.node import Node
import sys
import time
from adafruit_pca9685 import PCA9685
from adafruit_pca9685 import PWMChannel
import board
from busio import I2C

from std_msgs.msg import Float32
from subwoofer_interfaces.msg import ServoAngles
from subwoofer_interfaces.srv import ServoMotion


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




class Subwoofer(Node):
    def __init__(self):
        super().__init__("subwoofer")
        self.declare_parameter("simulated", True)

        self.servos = {
            "hip_1": Servo(5),
            "hip_2": Servo(4),
            "hip_3": Servo(7),
            "hip_4": Servo(6),

            "upper_1": Servo(2, 80),
            "upper_2": Servo(3, -80),
            "upper_3": Servo(13, 60),
            "upper_4": Servo(12, -80),

            "lower_1": Servo(0, -90),
            "lower_2": Servo(1, 90),
            "lower_3": Servo(15, -100),
            "lower_4": Servo(14, 100),
        }

        self.pub_servo = self.create_publisher(ServoAngles,
                                               "subwoofer/servos/angles",
                                               10)
        self.servo_timer = self.create_timer(0.02,
                                             self.update_servos)
        self.servo_control = self.create_service(ServoMotion,
                                                 "subwoofer/servos/update",
                                                 self.on_servo_request)
        

    def shutdown(self):
        for servo in self.servos.values():
            servo.stop()

    def update_servos(self):
        msg = ServoAngles()

        for i, servo in enumerate(self.servos.values()):
            servo.update_servo()
            msg.angles[i] = servo.angle
        
        self.pub_servo.publish(msg)


    def on_servo_request(self, request, response):
        if request.servo_id < 0 or request.servo_id > len(self.servos.values()):
            response.is_valid = False
        else:
            servo: Servo = list(self.servos.values())[request.servo_id]
            if request.velocity == 0:
                servo.stop()
            else:
                servo.set_target(request.angle, request.velocity)
            response.is_valid = True
        return response





def main(args=None):
    rclpy.init(args=args)
    node = Subwoofer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        node.destroy_node()
