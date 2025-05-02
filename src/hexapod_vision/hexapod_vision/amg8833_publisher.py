import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
import board
import busio
import adafruit_amg88xx

class AMG8833Publisher(Node):
    def __init__(self):
        super().__init__('amg8833_publisher')
        self.publisher_ = self.create_publisher(Image, 'thermal_image', 10)
        self.bridge = CvBridge()

        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

        self.timer = self.create_timer(0.2, self.publish_image)  # 5 Hz

    def publish_image(self):
        pixels = np.array(self.sensor.pixels)
        
        # Resize the image to 720x720 for better resolution
        resized = cv2.resize(pixels, (720, 720), interpolation=cv2.INTER_CUBIC)

        # Flip the image vertically
        flipped = cv2.flip(resized, 0)

        # Normalize the image for display purposes
        normalized = cv2.normalize(flipped, None, 0, 255, cv2.NORM_MINMAX)
        
        image = np.uint8(normalized)

        # Apply a color map to the image for better visualization
        color_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)

        # Convert the image to ROS message format
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'amg8833_frame'
        
        # Publish the image
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AMG8833Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
