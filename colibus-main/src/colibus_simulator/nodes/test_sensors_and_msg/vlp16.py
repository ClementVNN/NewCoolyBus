import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, NavSatFix
import random
import time


class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Publishers for sensors
        self.lidar_pub = self.create_publisher(LaserScan, '/sensor/lidar', 10)
        self.camera_pub = self.create_publisher(Image, '/sensor/camera', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensor/imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/sensor/gps', 10)

        # Subscribers for sensors
        self.create_subscription(LaserScan, '/sensor/lidar', self.lidar_callback, 10)
        self.create_subscription(Image, '/sensor/camera', self.camera_callback, 10)
        self.create_subscription(Imu, '/sensor/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/sensor/gps', self.gps_callback, 10)

        # Timers to publish data at intervals
        self.create_timer(0.1, self.publish_lidar_data)  # 10 Hz
        self.create_timer(0.2, self.publish_camera_data)  # 5 Hz
        self.create_timer(0.05, self.publish_imu_data)  # 20 Hz
        self.create_timer(1.0, self.publish_gps_data)  # 1 Hz

    # Mock data publishers
    def publish_lidar_data(self):
        msg = LaserScan()
        msg.header.frame_id = 'lidar_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.time_increment = 0.001
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [random.uniform(0.1, 10.0) for _ in range(int((msg.angle_max - msg.angle_min) / msg.angle_increment))]
        self.lidar_pub.publish(msg)
        self.get_logger().info('Published LiDAR data')

    def publish_camera_data(self):
        msg = Image()
        msg.header.frame_id = 'camera_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 3 * msg.width
        msg.data = [random.randint(0, 255) for _ in range(msg.height * msg.step)]
        self.camera_pub.publish(msg)
        self.get_logger().info('Published Camera data')

    def publish_imu_data(self):
        msg = Imu()
        msg.header.frame_id = 'imu_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation.x = random.uniform(-1.0, 1.0)
        msg.orientation.y = random.uniform(-1.0, 1.0)
        msg.orientation.z = random.uniform(-1.0, 1.0)
        msg.orientation.w = random.uniform(-1.0, 1.0)
        msg.angular_velocity.x = random.uniform(-1.0, 1.0)
        msg.angular_velocity.y = random.uniform(-1.0, 1.0)
        msg.angular_velocity.z = random.uniform(-1.0, 1.0)
        msg.linear_acceleration.x = random.uniform(-10.0, 10.0)
       
