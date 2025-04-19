import time
import pigpio
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C and PCA9685 drivers
i2c = busio.I2C(SCL, SDA)
pca_1 = PCA9685(i2c, address=0x40)
pca_2 = PCA9685(i2c, address=0x41)
pca_1.frequency = 50
pca_2.frequency = 50

# PWM input pins from Radiolink AT10II
CH1_GPIO = 17  # Forward/backward
CH2_GPIO = 27  # Left/right
CH5_GPIO = 22  # Gait toggle

# Servo pulse range and angle mapping
MIN_ANGLE = 0
MAX_ANGLE = 180
MIN_PWM_US = 500
MAX_PWM_US = 2500

# Pigpio setup
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon.")

pi.set_mode(CH1_GPIO, pigpio.INPUT)
pi.set_mode(CH2_GPIO, pigpio.INPUT)
pi.set_mode(CH5_GPIO, pigpio.INPUT)

def angle_to_duty_cycle(angle, freq=50):
    pulse_us = MIN_PWM_US + (angle / 180.0) * (MAX_PWM_US - MIN_PWM_US)
    return int((pulse_us / 1_000_000.0) * freq * 65536)

# Define servo channel layout
LEG_CHANNELS = [
    [3, 2, 1, 0],    # Leg 1 (PCA 0x40)
    [7, 6, 5, 4],    # Leg 2 (PCA 0x40)
    [15, 14, 13, 12],# Leg 3 (PCA 0x40)
    [3, 2, 1, 0],    # Leg 4 (PCA 0x41)
    [7, 6, 5, 4],    # Leg 5 (PCA 0x41)
    [11,10, 9, 8],   # Leg 6 (PCA 0x41)
]
LEG_PCAS = [pca_1, pca_1, pca_1, pca_2, pca_2, pca_2]

# Tripod groups: [0, 3, 4] and [1, 2, 5]
TRIPOD_GROUPS = [[0, 3, 4], [1, 2, 5]]

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

def move_forward_tripod(tripod_group, direction=0):
    step = 15  # Amount of lift/swing

    # Step 1: Raise tripod legs
    for leg in tripod_group:
        raise_leg(leg, step)
    time.sleep(0.3)

    # Step 2: Swing forward/backward/rotate
    for leg in tripod_group:
        if direction == 0:  # forward
            move_leg(leg, (90, 70, 100, 90))
        elif direction == 1:  # turn right
            move_leg(leg, (105, 70, 100, 90))
        elif direction == -1:  # turn left
            move_leg(leg, (75, 70, 100, 90))
        elif direction == 2:  # forward + right
            move_leg(leg, (105, 70, 100, 90))
        elif direction == -2:  # forward + left
            move_leg(leg, (75, 70, 100, 90))
    time.sleep(0.3)

    # Step 3: Lower tripod legs
    for leg in tripod_group:
        lower_leg(leg)
    time.sleep(0.3)


# Store latest pulse widths
pwm_values = {
    CH1_GPIO: 1500,
    CH2_GPIO: 1500,
    CH5_GPIO: 1000
}

def create_pwm_callback(gpio):
    def cbf(gpio, level, tick):
        if level == 1:
            cbf.start_tick = tick
        elif level == 0:
            pulse = pigpio.tickDiff(cbf.start_tick, tick)
            pwm_values[gpio] = pulse
    cbf.start_tick = 0
    return cbf

# Setup callbacks
cb1 = pi.callback(CH1_GPIO, pigpio.EITHER_EDGE, create_pwm_callback(CH1_GPIO))
cb2 = pi.callback(CH2_GPIO, pigpio.EITHER_EDGE, create_pwm_callback(CH2_GPIO))
cb5 = pi.callback(CH5_GPIO, pigpio.EITHER_EDGE, create_pwm_callback(CH5_GPIO))


# Helper to convert PWM pulsewidth to gait direction
def read_gait_input():
    ch1 = pwm_values[CH1_GPIO]
    ch2 = pwm_values[CH2_GPIO]
    ch5 = pwm_values[CH5_GPIO]

    forward = ch1 > 1600
    backward = ch1 < 1400
    right = ch2 > 1600
    left = ch2 < 1400
    gait_toggle = ch5 > 1500

    direction = 0
    if forward and right:
        direction = 2
    elif forward and left:
        direction = -2
    elif forward:
        direction = 0
    elif right:
        direction = 1
    elif left:
        direction = -1

    return direction, gait_toggle

# Debug loop
try:
    current_tripod = 0
    gait_active = False
    print("Starting Tripod Debug Gait Mode...")

    while True:
        direction, gait_toggle = read_gait_input()

        if gait_toggle and not gait_active:
            print("[Gait Enabled] Starting tripod gait...")
            gait_active = True

        if not gait_toggle and gait_active:
            print("[Gait Disabled] Stopping tripod gait...")
            gait_active = False

        if gait_active and direction is not None:
            tripod = TRIPOD_GROUPS[current_tripod]
            move_forward_tripod(tripod, direction)
            current_tripod = 1 - current_tripod  # Alternate tripods

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    cb1.cancel()
    cb2.cancel()
    cb5.cancel()
    pca_1.deinit()
    pca_2.deinit()
    pi.stop()
