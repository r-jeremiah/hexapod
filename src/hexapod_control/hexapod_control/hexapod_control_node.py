import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import time
import pigpio
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# Constants for tripod groups
TRIPOD_GROUPS = [[0, 3, 4], [1, 2, 5]]

# Constants for PWM and servo control
CH1_GPIO = 17  # Left/right
CH2_GPIO = 27  # Forward/backward
CH5_GPIO = 22  # Mode toggle
MIN_ANGLE = 0
MAX_ANGLE = 180
MIN_PWM_US = 500
MAX_PWM_US = 2500

# Initialize I2C and PCA9685 drivers
i2c = busio.I2C(SCL, SDA)
pca_1 = PCA9685(i2c, address=0x40)
pca_2 = PCA9685(i2c, address=0x41)
pca_1.frequency = 50
pca_2.frequency = 50

# Servo channel layout
LEG_CHANNELS = [
    [3, 2, 1, 0],    # Leg 1 (PCA 0x40)
    [7, 6, 5, 4],    # Leg 2 (PCA 0x40)
    [15, 14, 13, 12],# Leg 3 (PCA 0x40)
    [3, 2, 1, 0],    # Leg 4 (PCA 0x41)
    [7, 6, 5, 4],    # Leg 5 (PCA 0x41)
    [11, 10, 9, 8],  # Leg 6 (PCA 0x41)
]
LEG_PCAS = [pca_1, pca_1, pca_1, pca_2, pca_2, pca_2]

# Helper functions
def angle_to_duty_cycle(angle, freq=50):
    pulse_us = MIN_PWM_US + (angle / 180.0) * (MAX_PWM_US - MIN_PWM_US)
    return int((pulse_us / 1_000_000.0) * freq * 65536)

def move_leg(leg_index, positions):
    pca = LEG_PCAS[leg_index]
    coxa, femur, tibia, tarsus = LEG_CHANNELS[leg_index]
    angles = positions  # (coxa, femur, tibia, tarsus)
    for channel, angle in zip([coxa, femur, tibia, tarsus], angles):
        duty = angle_to_duty_cycle(angle)
        pca.channels[channel].duty_cycle = duty

def raise_leg(leg_index, step):
    move_leg(leg_index, (90, 70 - step, 100 + step, 90))

def lower_leg(leg_index):
    move_leg(leg_index, (90, 90, 90, 90))

class HexapodControlNode(Node):
    def __init__(self):
        super().__init__('hexapod_control_node')

        # Initialize pigpio for PWM input
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon.")

        self.pi.set_mode(CH1_GPIO, pigpio.INPUT)
        self.pi.set_mode(CH2_GPIO, pigpio.INPUT)
        self.pi.set_mode(CH5_GPIO, pigpio.INPUT)

        # Store latest PWM values
        self.pwm_values = {
            CH1_GPIO: 1500,
            CH2_GPIO: 1500,
            CH5_GPIO: 1000
        }

        # Setup PWM callbacks
        self.pi.callback(CH1_GPIO, pigpio.EITHER_EDGE, self.create_pwm_callback(CH1_GPIO))
        self.pi.callback(CH2_GPIO, pigpio.EITHER_EDGE, self.create_pwm_callback(CH2_GPIO))
        self.pi.callback(CH5_GPIO, pigpio.EITHER_EDGE, self.create_pwm_callback(CH5_GPIO))

        self.get_logger().info('Hexapod Control Node initialized in manual mode.')

    def create_pwm_callback(self, gpio):
        def cbf(gpio, level, tick):
            if level == 1:
                cbf.start_tick = tick
            elif level == 0:
                pulse = pigpio.tickDiff(cbf.start_tick, tick)
                self.pwm_values[gpio] = pulse
        cbf.start_tick = 0
        return cbf

    def read_gait_input(self):
        ch1 = self.pwm_values[CH1_GPIO]
        ch2 = self.pwm_values[CH2_GPIO]
        ch5 = self.pwm_values[CH5_GPIO]

        # Determine directions based on thresholds
        forward = ch2 < 1450
        backward = ch2 > 1550
        right = ch1 > 1600
        left = ch1 < 1400

        direction = None
        if forward and right:
            direction = 2
            direction_str = "Forward + Right"
        elif forward and left:
            direction = -2
            direction_str = "Forward + Left"
        elif forward:
            direction = 0
            direction_str = "Forward"
        elif backward and right:
            direction = 3
            direction_str = "Backward + Right"
        elif backward and left:
            direction = -3
            direction_str = "Backward + Left"
        elif backward:
            direction = -4
            direction_str = "Backward"
        elif right:
            direction = 1
            direction_str = "Right"
        elif left:
            direction = -1
            direction_str = "Left"
        else:
            direction_str = "None"

        # Log the direction as a string
        self.get_logger().info(f"Direction: {direction_str}, CH1 (Left/Right): {ch1}, CH2 (Forward/Backward): {ch2}, CH5 (Mode Toggle): {ch5}")

        return direction, ch5

    def run_manual_mode(self):
        try:
            while rclpy.ok():
                direction, ch5 = self.read_gait_input()

                # Manual mode logic
                if ch5 >= 1700:  # Manual mode
                    self.get_logger().info(f"[MANUAL MODE] Direction: {direction}")
                    for leg_index in range(6):
                        raise_leg(leg_index, 10)
                        time.sleep(0.1)
                        lower_leg(leg_index)

                time.sleep(0.05)
        except KeyboardInterrupt:
            self.get_logger().info("Manual mode stopped by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        self.pi.stop()
        pca_1.deinit()
        pca_2.deinit()

def main(args=None):
    rclpy.init(args=args)
    node = HexapodControlNode()
    node.run_manual_mode()
    node.destroy_node()
    rclpy.shutdown()