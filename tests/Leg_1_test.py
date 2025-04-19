import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import pigpio

# Servo channel definitions (on PCA9685)
COXA_SERVO = 15
FEMUR_SERVO = 14
TIBIA_SERVO = 13
TARSUS_SERVO = 12

# GPIO pin definitions (Radiolink channels)
CH1_GPIO = 17  # Gait (femur, tibia,tarsus)
CH2_GPIO = 27  # Coxa torsion
CH5_GPIO = 22  # Gait toggle

# RC input pulse range (microseconds)
MIN_RC = 1000
MAX_RC = 2000

# Servo angle range
MIN_ANGLE = 0
MAX_ANGLE = 180

# Servo pulse range for PCA9685
MIN_PWM_US = 500
MAX_PWM_US = 2500

# Utility: RC PWM to angle
def rc_to_angle(pw):
    return max(MIN_ANGLE, min(MAX_ANGLE, (pw - MIN_RC) * 180.0 / (MAX_RC - MIN_RC)))

# Utility: Angle to PCA9685 duty cycle
def angle_to_duty_cycle(angle, freq=50):
    pulse_us = MIN_PWM_US + (angle / 180.0) * (MAX_PWM_US - MIN_PWM_US)
    return int((pulse_us / 1_000_000.0) * freq * 65536)

# Class to measure PWM input using edge timing
class PWMReader:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio
        self._high_tick = None
        self._pwidth = 1500
        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        if level == 1:
            self._high_tick = tick
        elif level == 0 and self._high_tick is not None:
            self._pwidth = pigpio.tickDiff(self._high_tick, tick)

    def pulse_width(self):
        return self._pwidth

    def cancel(self):
        self._cb.cancel()

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

# Initialize PWM readers
ch1_reader = PWMReader(pi, CH1_GPIO)
ch2_reader = PWMReader(pi, CH2_GPIO)
ch5_reader = PWMReader(pi, CH5_GPIO)

# Initialize I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# Set servos to 0 degrees slowly
print("Setting servos to 0° slowly over 20 seconds...")
for deg in range(0, 1):  # already at 0, keep stable
    duty = angle_to_duty_cycle(deg, pca.frequency)
    for ch in [COXA_SERVO, FEMUR_SERVO, TIBIA_SERVO, TARSUS_SERVO]:
        pca.channels[ch].duty_cycle = duty
    time.sleep(0.1)
time.sleep(20)

print("Starting control loop...")
gait_active = False

try:
    while True:
        ch1_pw = ch1_reader.pulse_width()
        ch2_pw = ch2_reader.pulse_width()
        ch5_pw = ch5_reader.pulse_width()

        # Toggle gait based on CH5 threshold
        gait_active = ch5_pw > 1500

        if gait_active:
            # Example predefined gait angles
            coxa_angle = rc_to_angle(ch1_pw)
            femur_angle = 90 + 90 * ((ch1_pw - 1500) / 500.0)  # Example gait swing
            tibia_angle = 90 - 90 * ((ch1_pw - 1500) / 500.0)

            pca.channels[COXA_SERVO].duty_cycle = angle_to_duty_cycle(coxa_angle)
            pca.channels[FEMUR_SERVO].duty_cycle = angle_to_duty_cycle(femur_angle)
            pca.channels[TIBIA_SERVO].duty_cycle = angle_to_duty_cycle(tibia_angle)
        else:
            # Hold neutral
            for ch in [COXA_SERVO, FEMUR_SERVO, TIBIA_SERVO]:
                pca.channels[ch].duty_cycle = angle_to_duty_cycle(90)

        # Tarsus is always active
        tarsus_angle = rc_to_angle(ch2_pw)
        pca.channels[TARSUS_SERVO].duty_cycle = angle_to_duty_cycle(tarsus_angle)

        time.sleep(0.02)

except KeyboardInterrupt:
    print("Interrupted. Stopping...")

finally:
    pca.deinit()
    pi.stop()
    ch1_reader.cancel()
    ch2_reader.cancel()
    ch5_reader.cancel()
