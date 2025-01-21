std_msgs/Header header

# Steering angle [deg]
float64 steer_ang

# Linear speed [Km/h]. Only used in SPEED_CONTROL mode
float64 lin_speed

# Desired acceleration [m/s^2]. Used in SPEED_CONTROL and ACCEL_CONTROL
float64 accel

# Brake strength [0-200]. Has priority over all other references
float64 brake_strength

# Throttle [0-200]. Only used in PEDAL_CONTROL mode
float64 throttle
