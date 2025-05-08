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
    try:
        # Validate leg_index
        if leg_index < 0 or leg_index >= len(LEG_PCAS):
            raise IndexError(f"Invalid leg_index: {leg_index}. Must be between 0 and {len(LEG_PCAS) - 1}.")

        pca = LEG_PCAS[leg_index]
        coxa, femur, tibia, tarsus = LEG_CHANNELS[leg_index]
        for channel, angle in zip([coxa, femur, tibia, tarsus], positions):
            duty = angle_to_duty_cycle(angle)
            pca.channels[channel].duty_cycle = duty
    except IndexError as e:
        print(f"IndexError: {e}")  # Replace with appropriate logging if needed
    except Exception as e:
        print(f"I²C error on leg {leg_index}: {e}")  # Replace with appropriate logging if needed
        reset_gpio.reset_pca()  # Reset PCA boards to recover

def move_leg_slowly(leg_index, current_positions, target_positions, step_size=5, delay_between_steps=0.01, sequence=None, logger=None):
    """
    Gradually move each joint to the target position in the specified sequence.
    :param leg_index: Index of the leg to move.
    :param current_positions: Current positions of the joints.
    :param target_positions: Target positions of the joints.
    :param step_size: Step size for gradual movement.
    :param delay_between_steps: Delay between each step for smoother movement.
    :param sequence: Desired order of joint movement (e.g., ["femur", "tibia", "tarsus", "coxa"]).
    :param logger: Logger instance for logging messages.
    """
    if sequence is None:
        sequence = ["femur", "tibia", "tarsus", "coxa"]

    joint_map = {"coxa": 0, "femur": 1, "tibia": 2, "tarsus": 3}

    for joint in sequence:
        joint_index = joint_map[joint]
        current = current_positions[joint_index]
        target = target_positions[joint_index]

        if logger:
            logger.info(f"Moving joint {joint} (index {joint_index}) of leg {leg_index} from {current} to {target}")

        while current != target:
            if current < target:
                current = min(current + step_size, target)
            elif current > target:
                current = max(current - step_size, target)

            # Update the joint position
            duty = angle_to_duty_cycle(current)
            LEG_PCAS[leg_index].channels[LEG_CHANNELS[leg_index][joint_index]].duty_cycle = duty

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

        # Add a dictionary to track the last sent positions
        self.default_positions = (90, 105, 10, 85)  # Default positions for all legs
        self.last_positions = {leg_index: self.default_positions for leg_index in range(6)}  # Initialize with default positions

        # Replace threading with a ROS timer
        self.create_timer(0.02, self.hold_default_positions)

        # Add a timer for gait execution
        self.gait_timer = self.create_timer(0.1, self.execute_gait_step)
        self.current_step = None

        # Add a timer for manual mode
        self.manual_mode_timer = self.create_timer(0.05, self.run_manual_mode)

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
        """Run manual mode logic periodically."""
       
        # Define step_size and delay_between_steps for manual mode
        step_size = 5  # Define a step size for raising the leg
        delay_between_steps = 0.02  # Faster delay for quicker response in manual mode
        delay_between_steps_execution = 0.05
        direction, ch5 = self.read_gait_input()

        # Determine the gait based on CH5 value
        # Determine the gait based on CH5 value
        if ch5 >= 1090 and ch5 < 1450:  # Ripple Gait
            gait = "ripple"
            if direction == 0:  # Forward
                gait_sequence = [[2], [1], [0], [5], [4], [3]]
            elif direction == -4:  # Backward
                gait_sequence = [[3], [4], [5], [0], [1], [2]]
            elif direction == 1:  # Right
                gait_sequence = [[1], [2, 5], [3, 0], [4]]
            elif direction == -1:  # Left
                gait_sequence = [[4], [3, 0], [2, 5], [1]]
            elif direction == 2:  # Forward + Right
                gait_sequence = [[0, 4], [1, 5], [2, 3]]
            elif direction == -2:  # Forward + Left
                gait_sequence = [[1, 5], [0, 4], [2, 3]]
            elif direction == 3:  # Backward + Right
                gait_sequence = [[1, 5], [0, 4], [2, 3]]
            elif direction == -3:  # Backward + Left
                gait_sequence = [[0, 4], [1, 5], [2, 3]]
        elif ch5 >= 1450 and ch5 < 1550:  # Wave Gait
                gait = "wave"
                if direction == 0:  # Forward
                    gait_sequence = [[1], [2], [3], [4], [5], [0]]
                elif direction == -4:  # Backward
                    gait_sequence = [[0], [5], [4], [3], [2], [1]]
                elif direction == 1:  # Right
                    gait_sequence = [[1], [2], [3], [4], [5], [0]]
                elif direction == -1:  # Left
                    gait_sequence = [[4], [3], [2], [1], [0], [5]]
                elif direction == 2:  # Forward + Right
                    gait_sequence = [[0], [4], [1], [5], [2], [3]]
                elif direction == -2:  # Forward + Left
                    gait_sequence = [[1], [5], [0], [4], [2], [3]]
                elif direction == 3:  # Backward + Right
                    gait_sequence = [[1], [5], [0], [4], [2], [3]]
                elif direction == -3:  # Backward + Left
                    gait_sequence = [[0], [4], [1], [5], [2], [3]]
        elif ch5 >= 1600:  # Tripod Gait
            gait = "tripod"
            if direction == 0:  # Forward
                gait_sequence = [[0, 3, 5], [1, 2, 4]]
            elif direction == -4:  # Backward
                gait_sequence = [[1, 2, 4], [0, 3, 5]]
            elif direction == 1:  # Right
                gait_sequence = [[0, 2, 4], [1, 3, 5]]
            elif direction == -1:  # Left
                gait_sequence = [[1, 3, 5], [0, 2, 4]]
            elif direction == 2:  # Forward + Right
                gait_sequence = [[0, 4], [1, 5], [2, 3]]
            elif direction == -2:  # Forward + Left
                gait_sequence = [[1, 5], [0, 4], [2, 3]]
            elif direction == 3:  # Backward + Right
                gait_sequence = [[1, 5], [0, 4], [2, 3]]
            elif direction == -3:  # Backward + Left
                gait_sequence = [[0, 4], [1, 5], [2, 3]]
        else:
            gait = None
            gait_sequence = []

        # Define custom angles for each direction and leg
        custom_angle_offsets = {
            0: {  # Forward
                0: (-10, 45, 35, 10),  # Leg 1
                1: (-10, 45, 35, 10),  # Leg 2
                2: (-10, 45, 35, 10),  # Leg 3
                3: (10, 45, 35, 10),  # Leg 4
                4: (10, 45, 35, 10),  # Leg 5
                5: (10, 45, 35, 10),  # Leg 6
            },
            -4: {  # Backward
                0: (10, 45, 35, 10),  # Leg 1
                1: (10, 45, 35, 10),  # Leg 2
                2: (10, 45, 35, 10),  # Leg 3
                3: (-10, 45, 35, 10),  # Leg 4
                4: (-10, 45, 35, 10),  # Leg 5
                5: (-10, 45, 35, 10),  # Leg 6
            },
            1: {  # Right
                0: (25, 45, 35, 10),  # Leg 1
                1: (25, 45, 35, 10),  # Leg 2
                2: (25, 45, 35, 10),  # Leg 3
                3: (25, 45, 35, 10),  # Leg 4
                4: (25, 45, 35, 10),  # Leg 5
                5: (25, 45, 35, 10),  # Leg 6
            },
            -1: {  # Left
                0: (-25, 45, 35, 10),  # Leg 1
                1: (-25, 45, 35, 10),  # Leg 2
                2: (-25, 45, 35, 10),  # Leg 3
                3: (-25, 45, 35, 10),  # Leg 4
                4: (-25, 45, 35, 10),  # Leg 5
                5: (-25, 45, 35, 10),  # Leg 6
            },
            2: {  # Forward + Right
                0: (-15, 45, 35, 10),  # Leg 1
                1: (-15, 45, 35, 10),  # Leg 2
                2: (-15, 45, 35, 10),  # Leg 3
                3: (5, 45, 35, 10),  # Leg 4
                4: (5, 45, 35, 10),  # Leg 5
                5: (5, 45, 35, 10),  # Leg 6
            },
            -2: {  # Forward + Left
                0: (-5, 45, 35, 10),  # Leg 1
                1: (-5, 45, 35, 10),  # Leg 2
                2: (-5, 45, 35, 10),  # Leg 3
                3: (-15, 45, 35, 10),  # Leg 4
                4: (-15, 45, 35, 10),  # Leg 5
                5: (-15, 45, 35, 10),  # Leg 6
            },
            3: {  # Backward + Right
                0: (5, 45, 35, 10),  # Leg 1
                1: (5, 45, 35, 10),  # Leg 2
                2: (5, 45, 35, 10),  # Leg 3
                3: (-15, 45, 35, 10),  # Leg 4
                4: (-15, 45, 35, 10),  # Leg 5
                5: (-15, 45, 35, 10),  # Leg 6
            },
            -3: {  # Backward + Left
                0: (-15, 45, 35, 10),  # Leg 1
                1: (-15, 45, 35, 10),  # Leg 2
                2: (-15, 45, 35, 10),  # Leg 3
                3: (-5, 45, 35, 10),  # Leg 4
                4: (-5, 45, 35, 10),  # Leg 5
                5: (-5, 45, 35, 10),  # Leg 6
            },
        }

        self.get_logger().info(f"Manual mode is running {gait}")  # Log when the method is executed
       
        if gait and direction is not None:  # Only proceed if a valid gait and direction are provided
            self.is_moving = True  # Indicate that the hexapod is moving
            self.get_logger().info(f"Gait: {gait}, Direction: {direction}")

            # Execute the gait sequence
            for step in gait_sequence:
                self.get_logger().info(f"Executing step: {step}")

                # Parallelize raising legs
                for leg_index in step:
                    self.raise_leg(leg_index, step_size)
                time.sleep(delay_between_steps)

                # Parallelize moving legs
                for leg_index in step:
                    self.move_leg_to_direction(leg_index, direction, custom_angle_offsets)
                time.sleep(delay_between_steps)

                # Parallelize lowering legs
                for leg_index in step:
                    self.lower_leg(leg_index, step=step_size, delay=delay_between_steps)
                time.sleep(delay_between_steps)
                    
                

            self.is_moving = False  # Reset the flag after movement
        else:
            # Hold default positions if no valid gait or direction
            self.hold_default_positions()

        time.sleep(0.05)

        # except KeyboardInterrupt:
        #     self.get_logger().info("Manual mode stopped by user.")
        # finally:
        #     self.cleanup()

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
        try:
            # Stop pigpio
            if self.pi.connected:
                self.pi.stop()
                self.get_logger().info("Pigpio daemon stopped.")
                time.sleep(0.5)  # Add delay for pigpio cleanup

                # Deinitialize PCA9685 boards
                pca_1.deinit()
                time.sleep(0.5)  # Add delay after deinitializing pca_1
                pca_2.deinit()
                self.get_logger().info("PCA9685 boards deinitialized.")
                time.sleep(0.5)  # Add delay after deinitializing pca_2

                # Reset GPIO pins
                reset_gpio.reset_pin()
                time.sleep(0.5)  # Add delay after resetting GPIO pins
                reset_gpio.reset_pca()
                self.get_logger().info("GPIO pins and PCA boards reset.")
                time.sleep(0.5)  # Add delay after resetting PCA boards

        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        finally:
            self.get_logger().info("Cleanup completed.")

    def initialize_legs(self):
        # Initial positions for all legs
        initial_positions = (90, 140, 110, 55)  # (coxa, femur, tibia, tarsus)
        self.default_positions = (90, 105, 10, 85)  # Store final positions as default positions
        # Adjust step_size and delay_between_steps to control speed and smoothness
        step_size = 4  # Decrease step size for smoother movement, increase for faster movement
        delay_between_steps = 0.05 # Increase delay for smoother movement, decrease for faster movement

        # Move all legs to the initial position
        self.get_logger().info("Moving all legs to the initial position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, self.default_positions, initial_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to initial positions: {initial_positions}")

        # Wait for 10 seconds
        time.sleep(3)

        # Move all legs to the final position
        self.get_logger().info("Moving all legs to the default position...")
        for leg_index in range(6):
            move_leg_slowly(leg_index, initial_positions, self.default_positions, step_size, delay_between_steps)
            self.get_logger().info(f"Leg {leg_index} moved to default positions: {self.default_positions}")
            self.get_logger().info(f"Leg {leg_index} Tarsus angle: {self.default_positions[3]}")

    def lower_leg(self, leg_index, step=6, delay=0.05):
        """
        Gradually lowers the leg to its default positions.
        :param leg_index: Index of the leg to lower.
        :param step: Step size for gradual lowering.
        :param delay: Delay between each step for smoother movement.
        """
        coxa, femur, tibia, tarsus = self.default_positions
        current_positions = self.last_positions.get(leg_index, self.default_positions)

        # Gradually move the leg to the default positions
        move_leg_slowly(leg_index, current_positions, (coxa, femur, tibia, tarsus), step, delay)

        # Update the last known positions
        self.last_positions[leg_index] = self.default_positions

    def raise_leg(self, leg_index, step):
        coxa, femur, tibia, tarsus = self.default_positions
        move_leg(leg_index, (coxa, femur - step, tibia + step, tarsus))

    def move_leg_to_direction(self, leg_index, direction, custom_angle_offsets=None):
        angles = self.default_positions

        if custom_angle_offsets and direction in custom_angle_offsets and leg_index in custom_angle_offsets[direction]:
            offsets = custom_angle_offsets[direction][leg_index]
            angles = tuple(default + offset for default, offset in zip(self.default_positions, offsets))
        else:
            self.get_logger().warn(f"No custom offsets found for leg {leg_index} in direction {direction}. Using default positions.")

        # Move all joints of the leg in the specified sequence
        target_positions = angles
        current_positions = self.last_positions[leg_index]
        sequence = ["femur", "tibia", "tarsus", "coxa"]  # Desired sequence
        move_leg_slowly(leg_index, current_positions, target_positions, step_size=5, delay_between_steps=0.01, sequence=sequence, logger=self.get_logger())

        # Update the last known positions
        self.last_positions[leg_index] = angles

    def hold_default_positions(self):
        """Continuously send default positions to all legs to hold their positions."""
        if not self.is_moving:  # Only hold positions if not moving
            # Check if PWM values indicate movement
            if any(value != 1500 for value in self.pwm_values.values()):
                return  # Exit if PWM indicates movement

            for leg_index in range(6):  # Iterate through all legs
                try:
                    # Only send positions if they have changed
                    if self.last_positions[leg_index] != self.default_positions:
                        self.get_logger().debug(f"Holding default positions for leg {leg_index}: {self.default_positions}")
                        move_leg(leg_index, self.default_positions)
                        self.last_positions[leg_index] = self.default_positions
                except Exception as e:
                    self.get_logger().warn(f"Failed to hold default positions for leg {leg_index}: {e}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to move leg {leg_index}: {e}")

    def execute_gait_step(self):
        """Execute one step of the gait sequence."""
        if self.is_moving and self.current_step is not None:
            for leg_index in self.current_step:
                self.raise_leg(leg_index, 10)
            time.sleep(0.1)

            for leg_index in self.current_step:
                self.move_leg_to_direction(leg_index, self.direction, self.custom_angle_offsets)

            for leg_index in self.current_step:
                self.lower_leg(leg_index)

            # Advance to the next step in the gait sequence
            self.current_step = self.gait_controller.step()

def main(args=None):
    rclpy.init(args=args)
    node = HexapodControlNode()

    try:
        # Initialize legs
        node.initialize_legs()

    # Spin the node to process timers and callbacks
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Log when the node is stopped by the user
        node.get_logger().info("Manual mode stopped by user.")
    finally:
        # Cleanup resources
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()