import RPi.GPIO as GPIO
import pigpio
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

class reset_gpio:
    def reset_pin():
        """
        Resets GPIO pins used in the hexapod control.
        """
        # Connect to pigpio daemon
        pi = pigpio.pi()
        if not pi.connected:
            print("Could not connect to pigpio daemon. Ensure it is running.")
            return

        # Reset GPIO pins used in the hexapod control
        GPIO_PINS = [17, 27, 22]  # CH1_GPIO, CH2_GPIO, CH5_GPIO
        for pin in GPIO_PINS:
            pi.set_mode(pin, pigpio.INPUT)  # Set pins to input mode
            pi.set_pull_up_down(pin, pigpio.PUD_OFF)  # Disable pull-up/down resistors

        # Stop pigpio daemon
        pi.stop()
        print("GPIO pins have been reset.")

    def reset_pca():
        """
        Initializes PCA9685 boards and sets all channels to a safe state.
        """
        print("Resetting PCA9685 boards...")
        i2c = busio.I2C(SCL, SDA)

        # Initialize PCA9685 boards
        pca_1 = PCA9685(i2c, address=0x40)  # Replace with your actual I2C address
        pca_2 = PCA9685(i2c, address=0x41)  # Replace with your actual I2C address

        # Set the PWM frequency
        pca_1.frequency = 50
        pca_2.frequency = 50

        # Set all channels to 0 duty cycle (safe state)
        for channel in range(16):
            pca_1.channels[channel].duty_cycle = 0
            pca_2.channels[channel].duty_cycle = 0

        # Deinitialize the PCA9685 boards
        pca_1.deinit()
        pca_2.deinit()
        print("PCA9685 boards reset complete.")

