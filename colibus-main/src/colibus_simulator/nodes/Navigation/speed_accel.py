import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from your_package_name.msg import (
    CarStatus,
    ClimaStatus,
    CarId,
    LogErr,
    PID,
    SteeringData,
    SpeedAccelData,
    BlkLights,
    Beep,
    LCDText
)
from your_package_name.srv import (
    SetControlMode,
    GetControlMode,
    PIDParams,
    GetBlStatus,
    GetRunMode
)

class VehicleControlNode(Node):

    def __init__(self):
        super().__init__('vehicle_control_node')
        
 
        # Additional subscriber for SpeedAccelData
        self.speed_accel_data_sub = self.create_subscription(
            SpeedAccelData,
            '/VEHICLE/control/speed_accel_data',
            self.handle_speed_accel_data,
            10
        )
        
        # Proportional gain for steering angle calculation
        self.K = 0.1  # Tune this value based on your requirements

    def handle_speed_accel_data(self, msg):
        # Extract acceleration and braking commands
        acceleration = msg.acceleration
        braking = msg.braking
        
        # Compute steering angle based on acceleration and braking
        steering_angle = self.K * (acceleration - braking)
        
        # Create and publish SteeringData message
        steering_msg = SteeringData()
        steering_msg.steering_angle = steering_angle
        self.steering_data_pub.publish(steering_msg)