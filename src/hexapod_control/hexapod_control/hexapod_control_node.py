import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
import time
import pigpio
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import math
from hexapod_control.reset_gpio import reset_gpio  # Import the reset function

# Constants for tripod groups
TRIPOD_GROUPS = [[0, 3, 5], [1, 2, 4]]

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
    [3, 2, 1, 0],    # Leg 0
    [7, 6, 5, 4],    # Leg 1
    [15, 14, 13, 12],# Leg 2
    [3, 2, 1, 0],    # Leg 3
    [7, 6, 5, 4],    # Leg 4
    [11, 10, 9, 8],  # Leg 5
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

def move_leg_slowly(leg_index, target_positions, step_size, delay_between_steps):
    # Get the current positions of the leg (assume starting at 90 degrees for all joints)
    current_positions = [90, 130, 90, 40]  # Default starting positions matching the initialize position
    coxa, femur, tibia, tarsus = LEG_CHANNELS[leg_index]

    # Gradually move each joint to the target position
    for i, (current, target) in enumerate(zip(current_positions, target_positions)):
        while current != target:
            if current < target:
                current = min(current + step_size, target)
            elif current > target:
                current = max(current - step_size, target)

            # Update the joint position
            duty = angle_to_duty_cycle(current)
            LEG_PCAS[leg_index].channels[LEG_CHANNELS[leg_index][i]].duty_cycle = duty

            # Delay between steps
            time.sleep(delay_between_steps)

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

        # Subscribe to IMU data for stabilization
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10
        )

        # Stabilization variables
        self.roll = 0.0
        self.pitch = 0.0

        self.get_logger().info('Hexapod Control Node initialized with stabilization.')

    class PWMCallback:
        def __init__(self, gpio, pwm_values):
            self.gpio = gpio
            self.pwm_values = pwm_values
            self.start_tick = 0

        def __call__(self, gpio, level, tick):
            if level == 1:
                self.start_tick = tick
            elif level == 0:
                pulse = pigpio.tickDiff(self.start_tick, tick)
                self.pwm_values[self.gpio] = pulse

    def create_pwm_callback(self, gpio):
        return self.PWMCallback(gpio, self.pwm_values)

    def imu_callback(self, msg):
        # Extract roll and pitch from the IMU quaternion
        q = msg.orientation
        self.roll, self.pitch = self.quaternion_to_euler(q.w, q.x, q.y, q.z)

    def quaternion_to_euler(self, w, x, y, z):
        # Convert quaternion to roll, pitch, and yaw
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        return roll, pitch

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
                    if direction is not None:  # Only proceed if a valid direction is provided
                        self.get_logger().info(f"[MANUAL MODE] Direction: {{direction_str}}")

                        # Calculate angles based on CH1 and CH2 inputs
                        ch1_angle = (self.pwm_values[CH1_GPIO] - 1500) / 500 * 45 + 90  # Map CH1 to angle
                        ch2_angle = (self.pwm_values[CH2_GPIO] - 1500) / 500 * 45 + 90  # Map CH2 to angle

                        # Apply stabilization adjustments
                        adjustment = self.calculate_stabilization_adjustment()

                        # Move Tripod Group 1 (Legs 0, 3, 4)
                        self.get_logger().info("Moving Tripod Group 1")
                        self.get_logger().info(f"Raising legs in Tripod Group 1: {TRIPOD_GROUPS[0]}")
                        for leg_index in TRIPOD_GROUPS[0]:
                            self.raise_leg(leg_index, 10 + adjustment, ch1_angle, ch2_angle)
                        time.sleep(0.1)  # Allow time for the legs to raise
                        for leg_index in TRIPOD_GROUPS[0]:
                            self.move_leg_to_direction(leg_index, direction, adjustment)
                            self.lower_leg(leg_index, ch1_angle, ch2_angle)

                        # Move Tripod Group 2 (Legs 1, 2, 5)
                        self.get_logger().info("Moving Tripod Group 2")
                        self.get_logger().info(f"Raising legs in Tripod Group 2: {TRIPOD_GROUPS[1]}")
                        for leg_index in TRIPOD_GROUPS[1]:
                            self.raise_leg(leg_index, 10 + adjustment, ch1_angle, ch2_angle)
                        time.sleep(0.1)  # Allow time for the legs to raise
                        for leg_index in TRIPOD_GROUPS[1]:
                            self.move_leg_to_direction(leg_index, direction, adjustment)
                            self.lower_leg(leg_index, ch1_angle, ch2_angle)
                    else:
                        self.get_logger().info("[MANUAL MODE] No valid direction detected. Legs remain stationary.")

                time.sleep(0.05)
        except KeyboardInterrupt:
            self.get_logger().info("Manual mode stopped by user.")
        finally:
            self.cleanup()

    def calculate_stabilization_adjustment(self):
        # Calculate stabilization adjustment based on pitch
        if abs(math.degrees(self.pitch)) > 45:
            return 10 if self.pitch > 0 else -10
        return 0

    def cleanup(self):
        self.pi.stop()
        pca_1.deinit()
        pca_2.deinit()
        reset_gpio.reset_pin()
        reset_gpio.reset_pca()

    def initialize_legs(self):
        # Initial positions for all legs
        initial_positions = (90, 140, 90, 40)  # (coxa, femur, tibia, tarsus)
        self.default_positions = (90, 120, 5, 120)  # Store final positions as default positions
        step_size = 1
        delay_between_steps = 0.05

        # Move all legs to the initial position
        self.get_logger().info("Moving all legs to the initial position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, initial_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to initial positions: {initial_positions}")

        # Wait for 10 seconds
        time.sleep(10)

        # Move all legs to the final position
        self.get_logger().info("Moving all legs to the final position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, self.default_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to final positions: {self.default_positions}")

    def lower_leg(self, leg_index, ch1_angle=90, ch2_angle=90):
        """
        Lower the leg to its default position.
        :param leg_index: Index of the leg to lower.
        :param ch1_angle: Angle for the coxa joint (default 90).
        :param ch2_angle: Angle for the tarsus joint (default 90).
        """
        move_leg(leg_index, self.default_positions)

    def raise_leg(self, leg_index, step, ch1_angle=90, ch2_angle=90):
        """
        Raise the leg based on channel inputs.
        :param leg_index: Index of the leg to raise.
        :param step: Step height for raising the leg.
        :param ch1_angle: Angle derived from CH1 (left/right) input.
        :param ch2_angle: Angle derived from CH2 (forward/backward) input.
        """
        coxa, femur, tibia, tarsus = self.default_positions
        move_leg(leg_index, (ch1_angle, femur - step, tibia + step, ch2_angle))

    def move_leg_to_direction(self, leg_index, direction, adjustment):
        """
        Move a leg based on the direction and stabilization adjustment.
        :param leg_index: Index of the leg to move.
        :param direction: Direction of movement (e.g., forward, backward, left, right).
        :param adjustment: Stabilization adjustment based on IMU data.
        """
        coxa, femur, tibia, tarsus = self.default_positions

        if direction == 0:  # Forward
            move_leg(leg_index, (coxa, femur - 10, tibia + 10, tarsus))
        elif direction == -4:  # Backward
            move_leg(leg_index, (coxa, femur + 10, tibia - 10, tarsus))
        elif direction == 1:  # Right
            move_leg(leg_index, (coxa + 10, femur, tibia, tarsus))
        elif direction == -1:  # Left
            move_leg(leg_index, (coxa - 10, femur, tibia, tarsus))
        else:
            # No movement, return to default position
            move_leg(leg_index, self.default_positions)

        time.sleep(0.1)  # Allow time for the leg to move

def main(args=None):
    rclpy.init(args=args)
    node = HexapodControlNode()

    # Initialize legs
    node.initialize_legs()

    # Run manual mode
    node.run_manual_mode()

    node.destroy_node()
    rclpy.shutdown()