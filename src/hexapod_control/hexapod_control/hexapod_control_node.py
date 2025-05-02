import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import time
import pigpio
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import math
from hexapod_control.reset_gpio import reset_gpio  # Import the reset function
import threading

# Constants for tripod groups
TRIPOD_GROUPS = [[0, 2, 4], [1, 3, 5]]

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
pca_1.frequency = 60
pca_2.frequency = 60

# Servo channel layout
LEG_CHANNELS = [
    [3, 2, 1, 0],    # Leg 0
    [7, 6, 5, 4],    # Leg 1
    [15, 14, 13, 12],# Leg 2
    [3, 2, 1, 0],    # Leg 3
    [7, 6, 5, 4],    # Leg 4
    [11, 10, 9, 15],  # Leg 5
]
LEG_PCAS = [pca_1, pca_1, pca_1, pca_2, pca_2, pca_2]

# Helper functions
def angle_to_duty_cycle(angle, freq=60):
    # Clamp the angle to the valid range
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    
    # Calculate the pulse width in microseconds
    pulse_us = MIN_PWM_US + (angle / 180.0) * (MAX_PWM_US - MIN_PWM_US)
    
    # Convert the pulse width to a duty cycle value
    duty_cycle = (pulse_us / 1_000_000.0) * freq * 65536
    
    return int(duty_cycle)

def move_leg(leg_index, positions):
    pca = LEG_PCAS[leg_index]
    coxa, femur, tibia, tarsus = LEG_CHANNELS[leg_index]
    angles = positions  # (coxa, femur, tibia, tarsus)
    for channel, angle in zip([coxa, femur, tibia, tarsus], angles):
        duty = angle_to_duty_cycle(angle)
        pca.channels[channel].duty_cycle = duty

def move_leg_slowly(leg_index, current_positions, target_positions, step_size, delay_between_steps):
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

        self.get_logger().info('Hexapod Control Node initialized.')

        # Initialize the movement flag
        self.is_moving = False

        # Start a thread to continuously hold default positions
        self.holding_thread = threading.Thread(target=self.hold_default_positions, daemon=True)
        self.holding_thread.start()

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
        # self.get_logger().info(f"Direction: {direction_str}, CH1 (Left/Right): {ch1}, CH2 (Forward/Backward): {ch2}, CH5 (Mode Toggle): {ch5}")

        return direction, ch5

    def run_manual_mode(self):
        try:
            while rclpy.ok():
                direction, ch5 = self.read_gait_input()

                # Manual mode logic
                if ch5 >= 1700:  # Manual mode
                    if direction is not None:  # Only proceed if a valid direction is provided
                        self.is_moving = True  # Indicate that the hexapod is moving
                        self.get_logger().info(f"[MANUAL MODE] Direction: {direction}")

                        # Move Tripod Group 1 (Legs 0, 3, 4)
                        self.get_logger().info("Moving Tripod Group 1")
                        for leg_index in TRIPOD_GROUPS[0]:
                            self.raise_leg(leg_index, 10)
                        time.sleep(0.1)

                        for leg_index in TRIPOD_GROUPS[0]:
                            self.move_leg_to_direction(leg_index, direction)

                        for leg_index in TRIPOD_GROUPS[0]:
                            self.lower_leg(leg_index)

                        # Move Tripod Group 2 (Legs 1, 2, 5)
                        self.get_logger().info("Moving Tripod Group 2")
                        for leg_index in TRIPOD_GROUPS[1]:
                            self.raise_leg(leg_index, 10)
                        time.sleep(0.1)

                        for leg_index in TRIPOD_GROUPS[1]:
                            self.move_leg_to_direction(leg_index, direction)

                        for leg_index in TRIPOD_GROUPS[1]:
                            self.lower_leg(leg_index)

                        self.is_moving = False  # Reset the flag after movement
                    else:
                        # self.get_logger().info("[MANUAL MODE] No valid direction detected. Holding legs in default positions.")
                        # for leg_index in range(6):
                        #     move_leg(leg_index, self.default_positions)
                        self.hold_default_positions()

                time.sleep(0.05)

        except KeyboardInterrupt:
            self.get_logger().info("Manual mode stopped by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        self.pi.stop()
        pca_1.deinit()
        pca_2.deinit()
        reset_gpio.reset_pin()
        reset_gpio.reset_pca()

    def initialize_legs(self):
        # Initial positions for all legs
        initial_positions = (90, 140, 110, 60)  # (coxa, femur, tibia, tarsus)
        self.default_positions = (90, 105, 10, 80)  # Store final positions as default positions
        # Adjust step_size and delay_between_steps to control speed and smoothness
        step_size = 3  # Decrease step size for smoother movement, increase for faster movement
        delay_between_steps = 0.1 # Increase delay for smoother movement, decrease for faster movement

        # Move all legs to the initial position
        self.get_logger().info("Moving all legs to the initial position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, self.default_positions, initial_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to initial positions: {initial_positions}")

        # Wait for 10 seconds
        time.sleep(10)

        # Move all legs to the final position
        self.get_logger().info("Moving all legs to the final position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, initial_positions, self.default_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to final positions: {self.default_positions}")
            self.get_logger().info(f"Leg {leg_index} Tarsus angle: {self.default_positions[3]}")

    def lower_leg(self, leg_index):
        move_leg(leg_index, self.default_positions)

    def raise_leg(self, leg_index, step):
        coxa, femur, tibia, tarsus = self.default_positions
        move_leg(leg_index, (coxa, femur - step, tibia + step, tarsus))

    def move_leg_to_direction(self, leg_index, direction):
        coxa, femur, tibia, tarsus = self.default_positions

        if direction == 0:  # Forward
            move_leg(leg_index, (coxa +1, femur - 45, tibia + 35, tarsus +1))
        elif direction == -4:  # Backward
            move_leg(leg_index, (coxa +1, femur + 45, tibia - 35, tarsus +1))
        elif direction == 1:  # Right
            move_leg(leg_index, (coxa + 30, femur - 30, tibia + 20, tarsus +1))
        elif direction == -1:  # Left
            move_leg(leg_index, (coxa - 30, femur + 30, tibia - 20, tarsus +1))
        elif direction == 2:  # Forward + Right
            move_leg(leg_index, (coxa + 15, femur - 45, tibia + 35, tarsus +1))
        elif direction == -2:  # Forward + Left
            move_leg(leg_index, (coxa - 15, femur - 45, tibia + 35, tarsus +1))
        elif direction == 3:  # Backward + Right
            move_leg(leg_index, (coxa + 15, femur + 45, tibia - 35, tarsus +1))
        elif direction == -3:  # Backward + Left
            move_leg(leg_index, (coxa - 15, femur + 45, tibia - 35, tarsus +1))
        else:
            # No movement, ensure the leg holds its default position
            self.get_logger().info(f"Leg {leg_index} holding default position: {self.default_positions}")
            move_leg(leg_index, self.default_positions)

        time.sleep(0.2)  # Increase delay to make movement slower

    def hold_default_positions(self):
        """Continuously send default positions to all legs to hold their positions."""
        while rclpy.ok():  # Run as long as ROS is active
            if not self.is_moving:  # Only hold positions if not moving
                for leg_index in range(6):  # Iterate through all legs
                    pca = LEG_PCAS[leg_index]
                    coxa, femur, tibia, tarsus = LEG_CHANNELS[leg_index]

                    # Check if all channels for the leg are active
                    if all(pca.channels[channel].duty_cycle != 0 for channel in [coxa, femur, tibia, tarsus]):
                        move_leg(leg_index, self.default_positions)
                    else:
                        self.get_logger().warn(f"Leg {leg_index} has inactive PWM channels. Skipping.")

                time.sleep(0.02)  # Reduce delay for faster updates

def main(args=None):
    rclpy.init(args=args)
    node = HexapodControlNode()

    # Initialize legs
    node.initialize_legs()

    # Run manual mode
    node.run_manual_mode()

    node.destroy_node()
    rclpy.shutdown()