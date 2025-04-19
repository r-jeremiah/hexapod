import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
# import pigpio

# Servo channel definitions (on PCA9685)
COXA_SERVO_LEG6 = 11
FEMUR_SERVO_LEG6 = 10
TIBIA_SERVO_LEG6 = 9
TARSUS_SERVO_LEG6 = 8

COXA_SERVO_LEG5 = 7
FEMUR_SERVO_LEG5 = 6
TIBIA_SERVO_LEG5 = 5
TARSUS_SERVO_LEG5 = 4

COXA_SERVO_LEG4 = 3
FEMUR_SERVO_LEG4 = 2
TIBIA_SERVO_LEG4 = 1
TARSUS_SERVO_LEG4 = 0

COXA_SERVO_LEG3 = 15
FEMUR_SERVO_LEG3 = 14
TIBIA_SERVO_LEG3 = 13
TARSUS_SERVO_LEG3 = 12

COXA_SERVO_LEG2 = 7
FEMUR_SERVO_LEG2 = 6
TIBIA_SERVO_LEG2 = 5
TARSUS_SERVO_LEG2 = 4

COXA_SERVO_LEG1 = 3
FEMUR_SERVO_LEG1 = 2
TIBIA_SERVO_LEG1 = 1
TARSUS_SERVO_LEG1 = 0

# Servo angle range
MIN_ANGLE = 0
MAX_ANGLE = 180

# Servo pulse range for PCA9685
MIN_PWM_US = 500
MAX_PWM_US = 2500

# Utility: Angle to PCA9685 duty cycle
def angle_to_duty_cycle(angle, freq=50):
    pulse_us = MIN_PWM_US + (angle / 180.0) * (MAX_PWM_US - MIN_PWM_US)
    return int((pulse_us / 1_000_000.0) * freq * 65536)

# Initialize i2c 
i2c = busio.I2C(SCL, SDA)

# Initialize PCA9685_1
pca_1 = PCA9685(i2c, address=0x40)
pca_1.frequency = 50

# Initialize PCA9685_2
pca_2 = PCA9685(i2c, address=0x41)
pca_2.frequency = 50

try:
    step_size = 1       # Default step size for gradual movement (degrees)
    delay_between_steps = 0.05  # Default delay between each step (seconds)

    # Function to move a single servo to the target angle
    def move_servo(servo_channel, target_angle, step_size, delay_between_steps, pca, frequency):
        for current_angle in range(MIN_ANGLE, target_angle + 1, step_size):
            duty = angle_to_duty_cycle(current_angle, frequency)
            pca.channels[servo_channel].duty_cycle = duty
            time.sleep(delay_between_steps)

    # Function to move all servos of a leg to the target angle
    def move_leg(leg_channels, target_angle, step_size, delay_between_steps, pca, frequency):
        for current_angle in range(MIN_ANGLE, target_angle + 1, step_size):
            duty = angle_to_duty_cycle(current_angle, frequency)
            for ch in leg_channels:
                pca.channels[ch].duty_cycle = duty
            time.sleep(delay_between_steps)

    # Function to perform a continuous sweep for a single servo
    def sweep_servo(servo_channel, step_size, delay_between_steps, pca, frequency):
        while True:
            # Sweep from MIN_ANGLE to MAX_ANGLE
            for current_angle in range(MIN_ANGLE, MAX_ANGLE + 1, step_size):
                duty = angle_to_duty_cycle(current_angle, frequency)
                pca.channels[servo_channel].duty_cycle = duty
                time.sleep(delay_between_steps)
            # Sweep back from MAX_ANGLE to MIN_ANGLE
            for current_angle in range(MAX_ANGLE, MIN_ANGLE - 1, -step_size):
                duty = angle_to_duty_cycle(current_angle, frequency)
                pca.channels[servo_channel].duty_cycle = duty
                time.sleep(delay_between_steps)

    # Move a specific leg or servo based on user input
    leg_channels_list = [
        [COXA_SERVO_LEG1, FEMUR_SERVO_LEG1, TIBIA_SERVO_LEG1, TARSUS_SERVO_LEG1],
        [COXA_SERVO_LEG2, FEMUR_SERVO_LEG2, TIBIA_SERVO_LEG2, TARSUS_SERVO_LEG2],
        [COXA_SERVO_LEG3, FEMUR_SERVO_LEG3, TIBIA_SERVO_LEG3, TARSUS_SERVO_LEG3],
        [COXA_SERVO_LEG4, FEMUR_SERVO_LEG4, TIBIA_SERVO_LEG4, TARSUS_SERVO_LEG4],
        [COXA_SERVO_LEG5, FEMUR_SERVO_LEG5, TIBIA_SERVO_LEG5, TARSUS_SERVO_LEG5],
        [COXA_SERVO_LEG6, FEMUR_SERVO_LEG6, TIBIA_SERVO_LEG6, TARSUS_SERVO_LEG6]
    ]

    # Assign PCA for each leg
    pca_list = [pca_1, pca_1, pca_1, pca_2, pca_2, pca_2]

    while True:  # Continuous loop for setting up servos
        while True:
            print("\nSelect a leg to move (1-6) or type 0 to exit:")
            try:
                selected_leg = int(input("Enter leg number: ")) - 1  # Convert to zero-based index
                if selected_leg == -1:  # Exit condition
                    print("Exiting...")
                    exit(0)
                if 0 <= selected_leg < len(leg_channels_list):
                    break
                else:
                    print("Invalid leg number. Please select a number between 1 and 6.")
            except ValueError:
                print("Invalid input. Please enter a number.")

        leg_channels = leg_channels_list[selected_leg]
        pca = pca_list[selected_leg]

        while True:
            print("\nSelect an action:")
            print("1: Move the whole leg")
            print("2: Move an individual servo")
            print("0: Go back to leg selection")
            try:
                action = int(input("Enter action number (0-2): "))
                if action == 0:
                    break
                if action == 1:
                    while True:
                        try:
                            target_angle = int(input(f"Enter the target angle for leg {selected_leg + 1} (0-180): "))
                            if MIN_ANGLE <= target_angle <= MAX_ANGLE:
                                step_size = int(input("Enter the step size (1-10, smaller is slower): "))
                                delay_between_steps = float(input("Enter the delay between steps in seconds (e.g., 0.05): "))
                                move_leg(leg_channels, target_angle, step_size, delay_between_steps, pca, pca.frequency)
                                print(f"Moved leg {selected_leg + 1} to {target_angle} degrees.")
                                break
                            else:
                                print("Invalid angle. Please select a value between 0 and 180.")
                        except ValueError:
                            print("Invalid input. Please enter a valid number.")
                elif action == 2:
                    while True:
                        print("\nSelect a servo to move:")
                        print("1: Coxa")
                        print("2: Femur")
                        print("3: Tibia")
                        print("4: Tarsus")
                        print("0: Go back to action selection")
                        try:
                            selected_servo = int(input("Enter servo number (0-4): ")) - 1  # Convert to zero-based index
                            if selected_servo == -1:
                                break
                            if 0 <= selected_servo < len(leg_channels):
                                servo_channel = leg_channels[selected_servo]
                                while True:
                                    print("\nSelect an action:")
                                    print("1: Move to a specific angle")
                                    print("2: Perform a continuous sweep")
                                    print("0: Go back to servo selection")
                                    try:
                                        servo_action = int(input("Enter action number (0-2): "))
                                        if servo_action == 0:
                                            break
                                        if servo_action == 1:
                                            while True:
                                                try:
                                                    target_angle = int(input(f"Enter the target angle for servo {selected_servo + 1} (0-180): "))
                                                    if MIN_ANGLE <= target_angle <= MAX_ANGLE:
                                                        step_size = int(input("Enter the step size (1-10, smaller is slower): "))
                                                        delay_between_steps = float(input("Enter the delay between steps in seconds (e.g., 0.05): "))
                                                        move_servo(servo_channel, target_angle, step_size, delay_between_steps, pca, pca.frequency)
                                                        print(f"Moved servo {selected_servo + 1} of leg {selected_leg + 1} to {target_angle} degrees.")
                                                        break
                                                    else:
                                                        print("Invalid angle. Please select a value between 0 and 180.")
                                                except ValueError:
                                                    print("Invalid input. Please enter a valid number.")
                                        elif servo_action == 2:
                                            print(f"Starting continuous sweep for servo {selected_servo + 1} of leg {selected_leg + 1}. Press Ctrl+C to stop.")
                                            try:
                                                step_size = int(input("Enter the step size (1-10, smaller is slower): "))
                                                delay_between_steps = float(input("Enter the delay between steps in seconds (e.g., 0.05): "))
                                                sweep_servo(servo_channel, step_size, delay_between_steps, pca, pca.frequency)
                                            except KeyboardInterrupt:
                                                print("\nContinuous sweep stopped.")
                                        else:
                                            print("Invalid action. Please select 0, 1, or 2.")
                                    except ValueError:
                                        print("Invalid input. Please enter a number.")
                            else:
                                print("Invalid servo number. Please select a number between 1 and 4.")
                        except ValueError:
                            print("Invalid input. Please enter a number.")
                else:
                    print("Invalid action. Please select 0, 1, or 2.")
            except ValueError:
                print("Invalid input. Please enter a number.")

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")

finally:
    pca_1.deinit()
    pca_2.deinit()  # Ensure PCA devices are deinitialized when the program exits

