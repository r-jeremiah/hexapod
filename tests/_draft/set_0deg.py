import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
# import pigpio

# Servo channel definitions (on PCA9685)
COXA_SERVO_LEG6 = 12
FEMUR_SERVO_LEG6 = 11
TIBIA_SERVO_LEG6 = 10
TARSUS_SERVO_LEG6 = 9

COXA_SERVO_LEG5 = 8
FEMUR_SERVO_LEG5 = 7
TIBIA_SERVO_LEG5 = 6
TARSUS_SERVO_LEG5 = 5


COXA_SERVO_LEG4 = 4
FEMUR_SERVO_LEG4 = 3
TIBIA_SERVO_LEG4 = 2
TARSUS_SERVO_LEG4 = 1


COXA_SERVO_LEG3 = 15
FEMUR_SERVO_LEG3 = 14
TIBIA_SERVO_LEG3 = 13
TARSUS_SERVO_LEG3 = 12

COXA_SERVO_LEG2 = 8
FEMUR_SERVO_LEG2 = 7
TIBIA_SERVO_LEG2 = 6
TARSUS_SERVO_LEG2 = 5


COXA_SERVO_LEG1 = 4
FEMUR_SERVO_LEG1 = 3
TIBIA_SERVO_LEG1 = 2
TARSUS_SERVO_LEG1 = 1

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
    target_angle = 180  # Target angle to set the servos
    step_size = 1     # Step size for gradual movement (degrees)
    delay_between_steps = 0.05  # Delay between each step (seconds)

    # Function to move a single leg's servos to the target angle
    def move_leg(leg_channels, target_angle, step_size, delay_between_steps, pca, frequency):
        for current_angle in range(MIN_ANGLE, target_angle + 1, step_size):
            duty = angle_to_duty_cycle(current_angle, frequency)
            for ch in leg_channels:
                pca.channels[ch].duty_cycle = duty
            time.sleep(delay_between_steps)

    # Move each leg one at a time
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

    for leg_channels, pca in zip(leg_channels_list, pca_list):
        move_leg(leg_channels, target_angle, step_size, delay_between_steps, pca, pca.frequency)
        time.sleep(5)
    time.sleep(20)  # Keep the servos at the target position for 20 seconds

finally:
    pca_1.deinit()
    pca_2.deinit()  # Ensure pca_2 is also deinitialized

