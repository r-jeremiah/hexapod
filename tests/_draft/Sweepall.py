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

# Sweep all servos simultaneously to a target angle
def sweep_all_servos(pcas, start_angle, end_angle, duration=10.0):
    steps = 100
    sleep_time = duration / steps
    for step in range(steps + 1):
        angle = start_angle + (step / steps) * (end_angle - start_angle)
        pulse_us = angle_to_pwm_us(angle)
        for pca in pcas:
            for ch in range(16):
                duty_cycle = int((pulse_us / 1000000.0) * pca.frequency * 65536)
                pca.channels[ch].duty_cycle = duty_cycle
        time.sleep(sleep_time)

try:
    print("Starting continuous sweep of all servos...")
    while True:  # Loop indefinitely
        print("Sweeping from 0° to 180°...")
        sweep_all_servos([pca_1, pca_2], 0, 180, duration=5)
        print("Sweeping from 180° to 0°...")
        sweep_all_servos([pca_1, pca_2], 180, 0, duration=5)

finally:
    pca_1.deinit()
    pca_2.deinit()
    print("Drivers deinitialized.")