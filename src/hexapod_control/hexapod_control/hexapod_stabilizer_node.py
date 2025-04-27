import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import math

class HexapodStabilizerNode(Node):
    def __init__(self):
        super().__init__('hexapod_stabilizer_node')

        # Subscribe to IMU data from mpu6050_publisher
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10
        )

        # Publishers for leg positions
        self.leg_publishers = [
            self.create_publisher(Pose, f'leg_{i}/position', 10) for i in range(6)
        ]

        self.get_logger().info('Hexapod Stabilizer Node initialized.')

    def imu_callback(self, msg):
        # Extract roll and pitch from the IMU quaternion
        q = msg.orientation
        roll, pitch = self.quaternion_to_euler(q.w, q.x, q.y, q.z)

        self.get_logger().info(f"Roll: {math.degrees(roll):.2f}, Pitch: {math.degrees(pitch):.2f}")

        # Stabilize the hexapod if the pitch exceeds 45 degrees
        if abs(math.degrees(pitch)) > 45:
            self.stabilize(pitch)

    def quaternion_to_euler(self, w, x, y, z):
        # Convert quaternion to roll, pitch, and yaw
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch

    def stabilize(self, pitch):
        # Adjust leg positions based on pitch
        adjustment = 10 if pitch > 0 else -10

        for leg_index in range(6):
            pose = Pose()
            pose.position.z = adjustment  # Adjust the z-axis for stabilization
            self.leg_publishers[leg_index].publish(pose)

        self.get_logger().info(f"Stabilizing hexapod with adjustment: {adjustment}")

def main(args=None):
    rclpy.init(args=args)
    node = HexapodStabilizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()