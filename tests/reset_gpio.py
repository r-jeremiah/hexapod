import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

def reset_gpio():
    """
    Cleans up all GPIO pins and initializes PCA9685 boards to a safe state.
    """
    print("Resetting all GPIO pins...")
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
    GPIO.cleanup()  # Reset all GPIO pins
    print("GPIO cleanup complete.")

def reset_pca9685():
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

if __name__ == "__main__":
    reset_gpio()
    reset_pca9685()