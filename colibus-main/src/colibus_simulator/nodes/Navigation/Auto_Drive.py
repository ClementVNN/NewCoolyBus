import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('simple_auto_drive_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_twist)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return

        # Simple processing: draw speed and steering information
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, f'Speed: 0.2 m/s', (10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Steering: 0 rad/s', (10, 70), font, 1, (0, 255, 0), 2)

        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = 0.2  # Set constant linear speed
        twist.angular.z = 0.0  # Set zero angular speed (go straight)
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()