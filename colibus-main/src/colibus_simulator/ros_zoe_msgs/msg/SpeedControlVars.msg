std_msgs/Header header

# Whether a condition to stop the vehicle is active or not
bool stop_condition

# Reference acceleration [m/s^2]
float64 accel_ref

# Push force [N]
float64 push_force

# Total acceleration [N]
float64 total_accel

# Friction force [N]
float64 fric_force

# Gravitational force [N]
float64 grav_force

# Acceleration control value []
float64 accel_ctr_val

# Braking force [N]
float64 braking_force

# Resultant force [N]
float64 result_force

# Speed reference [km/h]
float64 spd_ref

# Brake extension reference [0-200]
float64 brake_ext_ref

# Acceleration extension reference [m/s^2]
float64 accel_ext_ref

# Brake reference for low-level control [0-200]
float64 brake_strength

# Gear reference position
uint8 gear_ref

# Throttle control value []
float64 throttle_ctr_val

# Pitch angle [rad]
float64 pitch_ang

# Current status of the vehicle
CarStatus stat
