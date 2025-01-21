import rclpy
from rclpy.node import Node
from std_msgs.msg import Header,Bool
from colibus_simulator.msg import (
    CarStatus, ClimaStatus, PID, SteeringData, SpeedAccelData, CarInterfaceStatus,
    CarInterfaceCommand, CarInterfaceConditions, SpeedControlVars, Forces,
     ControlRefs, BlkLights, Beep, LCDText
)
from colibus_simulator.msg import CarId, LogErr
from colibus_simulator.srv import (
    SetControlMode, GetControlMode, PIDParams, GetBlStatus, GetRunMode
)
import random
import datetime
import time
from typing import List



class VehicleSimulatorNode(Node):
    def __init__(self):
        super().__init__('vehicle_simulator_node')

        # Publishers for topics
        self.car_status_pub = self.create_publisher(CarStatus, '/VEHICLE/car/status', 10)
        self.clima_status_pub = self.create_publisher(ClimaStatus, '/VEHICLE/clima/status', 10)
        self.car_id_pub = self.create_publisher(CarId, '/car/id', 10)
        self.log_err_pub = self.create_publisher(LogErr, '/VEHICLE/log/err', 10)
        self.steer_ang_pid_pub = self.create_publisher(PID, '/VEHICLE/control/steer_ang_pid', 10)
        self.brake_pid_pub = self.create_publisher(PID, '/VEHICLE/control/brake_pid', 10)
        self.steering_data_pub = self.create_publisher(SteeringData, '/VEHICLE/control/steering_data', 10)
        self.speed_accel_data_pub = self.create_publisher(SpeedAccelData, '/VEHICLE/control/speed_accel_data', 10)
        
        self.blk_data_sub = self.create_subscription(BlkLights, '/VEHICLE/control/blk_lights', self.handle_blk_light,10)
        self.beep_data_sub = self.create_subscription(Beep, '/VEHICLE/control/beep',self.handle_lcd_text, 10)
        self.lcd_data_sub = self.create_subscription(LCDText, '/VEHICLE/control/LCD_text',self.handle_lcd_text, 10)
        self.set_auto_mode_sub = self.create_subscription(Bool, '/VEHICLE/control/set_auto_mode', self.handle_set_auto_mode, 10)
        self.hand_brake_sub = self.create_subscription(Bool, '/VEHICLE/control/hand_brake', self.handle_hand_brake, 10)


        # Services
        self.set_longitudinal_mode_srv = self.create_service(SetControlMode, '/VEHICLE/control/set_longitudinal_ctr_mode', self.set_longitudinal_control_mode)
        self.get_longitudinal_mode_srv = self.create_service(GetControlMode, '/VEHICLE/control/get_longitudinal_ctr_mode', self.get_longitudinal_control_mode)
        self.set_steering_mode_srv = self.create_service(SetControlMode, '/VEHICLE/control/set_steering_ctr_mode', self.set_steering_control_mode)
        self.get_steering_mode_srv = self.create_service(GetControlMode, '/VEHICLE/control/get_steering_ctr_mode', self.get_steering_control_mode)
        self.get_pid_params_srv = self.create_service(PIDParams, '/VEHICLE/control/get_PID_params', self.get_pid_params)
        self.get_bl_status_srv = self.create_service(GetBlStatus, '/VEHICLE/control/get_bl_status', self.get_blinking_lights_status)
        self.get_run_mode_srv = self.create_service(GetRunMode, '/VEHICLE/control/get_run_mode', self.get_run_mode)


        # Timers
        self.create_timer(1.0, self.publish_car_id)
        self.create_timer(1.0, self.publish_log_err)
     

        # Timer to publish messages periodically
        self.timer = self.create_timer(1.0, self.publish_topics)

    def publish_topics(self):
        self.publish_car_status()
        self.publish_clima_status()
        self.publish_car_id()
        self.publish_log_err()
        self.publish_pid('/VEHICLE/control/steer_ang_pid')
        self.publish_pid('/VEHICLE/control/brake_pid')
        self.publish_steering_data()
        self.publish_speed_accel_data()
        
    def handle_blk_light(self,msg) :
        self.get_logger().info(f'Light : {msg.side} "')
        
    def handle_beep_data(self,msg) :
        self.get_logger().info(f'Beep frequency: {msg.frequency}, Text: "{msg.duration}"')
    
    def handle_lcd_text(self, msg):
        self.get_logger().info(f'Received LCD text: Line {msg.line}, Text: "{msg.text}"')

    def handle_hand_brake(self, msg):
        if msg.data:
            self.get_logger().info('Hand brake activated')
        else:
            self.get_logger().info('No action taken (cannot deactivate hand brake)')

    def handle_set_auto_mode(self, msg):
        mode = 'AUTO' if msg.data else 'MANUAL'
        self.get_logger().info(f'Set auto mode: {mode}')

    def set_longitudinal_control_mode(self, request, response):
        mode_map = {0: 'PEDAL_CONTROL', 1: 'ACCEL_CONTROL', 2: 'SPEED_CONTROL'}
        mode_str = mode_map.get(request.mode, 'UNKNOWN')
        self.get_logger().info(f'Setting longitudinal control mode to {mode_str}')
        response.success = True
        return response

    def get_longitudinal_control_mode(self, request, response):
        response.mode = 2  # SPEED_CONTROL
        self.get_logger().info(f'Getting longitudinal control mode: SPEED_CONTROL')
        return response

    def set_steering_control_mode(self, request, response):
        mode_map = {0: 'STEERING_QUICK', 1: 'STEERING_MODERATE', 2: 'STEERING_SLOW'}
        mode_str = mode_map.get(request.mode, 'UNKNOWN')
        self.get_logger().info(f'Setting steering control mode to {mode_str}')
        response.success = True
        return response

    def get_steering_control_mode(self, request, response):
        response.mode = 0  # STEERING_QUICK
        self.get_logger().info(f'Getting steering control mode: STEERING_QUICK')
        return response

    def get_pid_params(self, request, response):
        self.get_logger().info('Getting PID parameters')
        response.steering_pid = PID(kp=1.0, ki=0.1, kd=0.01)  # Mock data
        response.brake_pid = PID(kp=1.5, ki=0.2, kd=0.02)  # Mock data
        return response

    def get_blinking_lights_status(self, request, response):
        self.get_logger().info('Getting blinking lights status')
        response.left = True  # Mock data
        response.right = False  # Mock data
        return response

    def get_run_mode(self, request, response):
        self.get_logger().info('Getting run mode')
        response.mode = 'NORMAL'  # Mock response
        return response

    def publish_car_status(self):
        msg = CarStatus()
        msg.lin_speed = random.uniform(0, 100)
        msg.accel = random.uniform(-5, 5)
        msg.steer_ang = random.uniform(-30, 30)
        msg.steer_vel = random.uniform(-10, 10)
        msg.brake_force = random.uniform(0, 200)
        msg.throttle = random.uniform(0, 250)
        msg.mode = random.choice([0, 1, 2])
        msg.gear = random.choice([0, 1, 2, 3])
        self.car_status_pub.publish(msg)
        self.get_logger().info(f'Published car status: {msg}')

    def publish_clima_status(self):
        msg = ClimaStatus()
        msg.air_dir = random.randint(0, 4)
        msg.ref_temp = random.uniform(16, 30)
        msg.fan_speed = random.randint(0, 8)
        self.clima_status_pub.publish(msg)
        self.get_logger().info(f'Published clima status: {msg}')

    def publish_car_id(self):
        msg = CarId()
        msg.id = random.randint(1, 100)
        msg.name = "TestVehicle"
        self.car_id_pub.publish(msg)
        self.get_logger().info(f'Published car ID: {msg}')

    def publish_log_err(self):
        msg = LogErr()
        msg.err_code = random.randint(0, 10)
        msg.name = "LOG_ERR_NO_OBD_DATA"
        msg.date_time = datetime.datetime.now().strftime('%d-%m-%Y %H:%M:%S')
        msg.check = "OBD"
        self.log_err_pub.publish(msg)
        self.get_logger().info(f'Published log error: {msg}')

    def publish_pid(self, topic_name):
        msg = PID()
        msg.val = random.uniform(-10, 10)
        msg.ref = random.uniform(-10, 10)
        msg.prop_term = random.uniform(-5, 5)
        msg.int_term = random.uniform(-5, 5)
        msg.der_term = random.uniform(-5, 5)
        msg.total = random.uniform(-15, 15)
        msg.pot_val = random.randint(0, 255)
        msg.err = random.uniform(-5, 5)
        publisher = getattr(self, topic_name.split('/')[-1] + '_pub', None)
        if publisher:
            publisher.publish(msg)
            self.get_logger().info(f'Published PID to {topic_name}: {msg}')

    def publish_steering_data(self):
        msg = SteeringData()
        msg.curr_acc = random.uniform(-30, 30)
        msg.ref_acc = random.uniform(-30, 30)
        self.steering_data_pub.publish(msg)
        self.get_logger().info(f'Published steering data: {msg}')

    def publish_speed_accel_data(self):
        msg = SpeedAccelData()
        msg.curr_acc = random.uniform(-5, 5)
        msg.ref_acc = random.uniform(-5, 5)
        msg.curr_spd = random.uniform(0, 100)
        msg.ref_spd = random.uniform(0, 100)
        msg.throttle = random.uniform(0, 200)
        msg.brake = random.uniform(0, 200)
        self.speed_accel_data_pub.publish(msg)
        self.get_logger().info(f'Published speed and acceleration data: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
