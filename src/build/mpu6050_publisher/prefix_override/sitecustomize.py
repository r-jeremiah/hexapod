import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hexapod/ros2_ws/src/install/mpu6050_publisher'
