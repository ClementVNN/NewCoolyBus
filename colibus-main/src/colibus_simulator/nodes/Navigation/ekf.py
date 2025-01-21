import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from your_package_name.msg import SpeedAccelData, SteeringData

class PathFollowingNode(Node):

    def __init__(self):
        super().__init__('path_following_node')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        
        # Publishers
        self.speed_pub = self.create_publisher(SpeedAccelData, '/VEHICLE/control/speed_accel_data', 10)
        self.steering_pub = self.create_publisher(SteeringData, '/VEHICLE/control/steering_data', 10)
        
        # Parameters
        self.waypoints = []
        self.current_pose = None
        self.Kp = 1.0  # Proportional gain for steering
        self.Kv = 0.5  # Proportional gain for velocity

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.follow_path()

    def path_callback(self, msg):
        self.waypoints = msg.poses

    def follow_path(self):
        if not self.waypoints or not self.current_pose:
            return

        # Find the closest waypoint
        min_dist = float('inf')
        closest_waypoint = None
        for wp in self.waypoints:
            # Calculate distance to waypoint
            dx = wp.pose.position.x - self.current_pose.position.x
            dy = wp.pose.position.y - self.current_pose.position.y
            dist = dx**2 + dy**2
            if dist < min_dist:
                min_dist = dist
                closest_waypoint = wp

        if closest_waypoint:
            # Calculate desired heading
            dx = closest_waypoint.pose.position.x - self.current_pose.position.x
            dy = closest_waypoint.pose.position.y - self.current_pose.position.y
            desired_heading = math.atan2(dy, dx)

            # Calculate current heading from orientation
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            _, _, current_heading = tf.transformations.euler_from_quaternion(quaternion)

            # Calculate lateral error
            lateral_error = math.sin(current_heading - desired_heading) * min_dist**0.5

            # PID for steering
            steering_angle = self.Kp * lateral_error

            # PID for velocity
            desired_speed = 1.0  # Set desired speed
            current_speed = self.current_pose.linear.x  # Assume available from odometry
            speed_error = desired_speed - current_speed
            acceleration = self.Kv * speed_error

            # Publish commands
            speed_msg = SpeedAccelData()
            speed_msg.acceleration = acceleration
            self.speed_pub.publish(speed_msg)

            steering_msg = SteeringData()
            steering_msg.steering_angle = steering_angle
            self.steering_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
