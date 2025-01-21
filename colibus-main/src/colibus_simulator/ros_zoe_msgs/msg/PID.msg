std_msgs/Header header

# Current steering angle [deg]
float64 val

# Steering reference for the controller [deg]
float64 ref

# Proportional term
float64 prop_term

# Integral term
float64 int_term

# Derivative term
float64 der_term

# Total control value
float64 total

# Potentiometer value applied on the steering module
uint8 pot_val

# Steering error [deg]
float64 err
