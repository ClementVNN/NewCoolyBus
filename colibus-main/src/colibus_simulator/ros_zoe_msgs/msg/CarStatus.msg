# Wheel indexes
uint8 FRONT_LEFT_WHEEL=0
uint8 FRONT_RIGHT_WHEEL=1
uint8 REAR_LEFT_WHEEL=2
uint8 REAR_RIGHT_WHEEL=3

# Seat belt vector indexes
uint8 FRONT_LEFT_SEAT_BELT=0
uint8 FRONT_RIGHT_SEAT_BELT=1
uint8 REAR_LEFT_SEAT_BELT=2
uint8 REAR_MIDDLE_SEAT_BELT=3
uint8 REAR_RIGHT_SEAT_BELT=4

# Doors vector indexes
uint8 FRONT_LEFT_DOOR=0
uint8 FRONT_RIGHT_DOOR=1
uint8 REAR_LEFT_DOOR=2
uint8 REAR_RIGHT_DOOR=3
uint8 TRUNK=4

# Blinking lights vector indexes
uint8 LEFT_BLK_LIGHTS=0
uint8 RIGHT_BLK_LIGHTS=1

# Header
std_msgs/Header header


# Wheel speeds [km/h]
float64[4] wheel_speeds  # [front_left, front_right, rear_left, rear_right]

# Wheel odometry [m]
float64[4] wheel_odometry  # [front_left, front_right, rear_left, rear_right]

# Vehicle dynamics
float64 lin_speed          # Vehicle's speed in [km/h]
float64 accel             # Vehicle's acceleration [m/s^2] (computed from linSpeed)
float64 steer_ang          # Steering angle [deg]
float64 steer_vel          # Steering velocity [deg/sec]
bool steer_manip           # Whether the steering wheel has been manipulated
float64 steer_torque       # Applied torque to the steering []
float64 brake_force        # Current force applied to the brake pedal [0-200]
bool brake_pedal          # [false: Brake pedal not pressed, true: Brake pedal pressed]
bool throttle_pedal       # [false: Throttle pedal not pressed, true: Throttle pedal pressed]
float64 throttle          # Current throttle value applied to the gas pedal [0-250]
uint8 gear                # Position of the gearbox [0: P, 1: R, 2: N, 3: D]

# Seat belts
bool[5] seatbelt          # Status of the seat belts [front_left, front_right, rear_left, rear_middle, rear_right]

# Doors
bool doors_closed         # [false: Some door open, true: All doors closed]
bool[5] doors             # Status of the doors [front_left, front_right, rear_left, rear_right, trunk]

# Blinking lights
bool[2] blk_lights        # Status of the blinking lights [left_bl, right_bl]

# Lights
uint8 lights              # Status of the main lights [0: off, 1: short, 2: long]
uint8 position_lights     # Status of the position lights [1: auto, 2: position]
bool fog_lights_front     # [false: Front fog lights are OFF, true: Fog lights are ON]
bool fog_lights_rear      # [false: Rear fog lights are OFF, true: Fog lights are ON]

# Charge connector
bool cap_charge_closed    # [false: The cap of the charge connector is open, true: The cap of the charge connector is closed]

# Ultrasound sensor zones
bool[10] front_zone       # Front ultrasound sensor zones numbered from left to right [1, 2, 3, 4, 5, 6, 7, 8, 9, 10], true means occupied
bool[10] rear_zone        # Rear ultrasound sensor zones numbered from left to right [1, 2, 3, 4, 5, 6, 7, 8, 9, 10], true means occupied

# Energy autonomy
uint16 autonomy           # Energy autonomy [Km]

# Mode
uint8 mode                # [0: Manual, 1: Auto, 2: Cooperative]

# Regenerative brake
float64 regenbrake        # Value indicating how much regenerative brake is being applied

# Wipers
uint8 frontwipers         # Status of the front wipers [0: off, 1: speed1, 2: speed2, 3: speed3, 4: single shot]
uint8 rearwipers          # Status of the rear wipers [0: off, 1: speed1, 2: speed2]

# Hand brake and hill start
bool handbrake            # [false: Hand brake not activated, true: Hand brake activated]
bool hillstart            # Hill start function working [0: off, 1: on]

# Communication status
bool can_comm_ok          # [false: Communication trouble with CAN bus, true: No problem of communication]
bool car_ifc_comm_ok      # [false: Communication trouble with car interface, true: No problem of communication]

# Control modes
uint8 long_control_mode   # Longitudinal control mode [0: Pedal, 1: Acceleration, 2: Speed]
uint8 str_control_mode    # Steering control mode [0: Quick, 1: Moderate, 2: Slow]
bool auto_str_ctr         # Steering control actuator in auto mode [true: yes, false: no]
bool auto_ttl_ctr         # Throttle control actuator in auto mode [true: yes, false: no]
bool auto_brk_ctr         # Brake control actuator in auto mode [true: yes, false: no]