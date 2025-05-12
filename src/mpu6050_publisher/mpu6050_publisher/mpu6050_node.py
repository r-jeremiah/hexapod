import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from mpu6050 import mpu6050
import math
from mpu6050_publisher.madgwickahrs import MadgwickAHRS
# from hexapod_vision.i2c_bus import i2c_bus  # Commented out as it could not be resolved

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.sensor = mpu6050(0x68)
        self.timer = self.create_timer(0.01, self.publish_data)  # 100 Hz update for smoother orientation

        # Initialize Madgwick Filter  # noqa: Madgwick
        self.ahrs = MadgwickAHRS()  # Initialize without arguments  # noqa: ahrs, Madgwick, AHRS
        self.ahrs.samplePeriod = 0.5  # Set sample period separately  # noqa: ahrs
        self.ahrs.beta = 0.1  # Set beta as needed  # noqa: ahrs

    def publish_data(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            self.get_logger().info(f"Raw Accelerometer: {accel}")
            self.get_logger().info(f"Raw Gyroscope: {gyro}")
        except OSError as e:
            self.get_logger().error(f"I2C Communication Error: {e}")
            return  # Skip this iteration if the sensor is not responding

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Convert accel from g to m/s²
        ax = accel['x'] * 9.80665
        ay = accel['y'] * 9.80665
        az = accel['z'] * 9.80665

        # Convert gyro from deg/s to rad/s
        gx = math.radians(gyro['x'])
        gy = math.radians(gyro['y'])
        gz = math.radians(gyro['z'])

        # Update Madgwick filter
        self.ahrs.update_imu([gx, gy, gz], [ax, ay, az])  # Pass gyro + accel as arrays

        # Get orientation quaternion
        q = self.ahrs.quaternion  # [w, x, y, z] order

        # Fill the IMU message
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]

        # Covariances
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
