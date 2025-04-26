import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from mpu6050 import mpu6050
import math
import sys
# sys.path.append('/home/hexapod/madgwick_py')

# from madgwickahrs import MadgwickAHRS

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.sensor = mpu6050(0x68)
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

    def publish_data(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Convert accel from g to m/s²
        msg.linear_acceleration.x = accel['x'] * 9.80665
        msg.linear_acceleration.y = accel['y'] * 9.80665
        msg.linear_acceleration.z = accel['z'] * 9.80665

        # Convert gyro from deg/s to rad/s
        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        # Orientation not estimated (yet)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Covariance (set non-zero diagonal for basic sensor fusion support)
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                           0.0, 0.01, 0.0,
                                           0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()