import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C and PCA9685 boards
i2c = busio.I2C(SCL, SDA)
pca_1 = PCA9685(i2c, address=0x40)
pca_2 = PCA9685(i2c, address=0x41)
pca_1.frequency = 50
pca_2.frequency = 50

# Convert angle to pulse width in microseconds
def angle_to_pwm_us(angle, min_us=500, max_us=2500):
    return min_us + (angle / 180.0) * (max_us - min_us)

# Set servo angle smoothly over 10 seconds
def smooth_servo_move(pca, channel, target_angle, duration=10.0):
    steps = 100
    sleep_time = duration / steps
    for step in range(steps + 1):
        angle = (step / steps) * target_angle
        pulse_us = angle_to_pwm_us(angle)
        duty_cycle = int((pulse_us / 1000000.0) * pca.frequency * 65536)
        pca.channels[channel].duty_cycle = duty_cycle
        time.sleep(sleep_time)

try:
    print("Setting servos on PCA9685 0x40 to 0°...")
    for ch in [i for i in range(16) if i not in {9, 10, 11, 12}]:
        print(f"Leg 0x40 - Channel {ch}: Moving to 0°")
        smooth_servo_move(pca_1, ch, 0)

    print("Setting servos on PCA9685 0x41 to 0°...")
    for ch in [i for i in range(16) if i not in {11, 12, 13, 14, 15}]:
        print(f"Leg 0x41 - Channel {ch}: Moving to 0°")
        smooth_servo_move(pca_2, ch, 0)

finally:
    pca_1.deinit()
    pca_2.deinit()
    print("All servos set to 0° and drivers deinitialized.")
