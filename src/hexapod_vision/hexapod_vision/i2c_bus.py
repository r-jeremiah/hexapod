# filepath: /home/hexapod/ros2_ws/src/hexapod_vision/hexapod_vision/i2c_bus.py
import board
import busio

# Create a shared I2C bus instance
i2c_bus = busio.I2C(board.SCL, board.SDA)