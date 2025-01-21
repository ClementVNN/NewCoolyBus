# Digital inputs vector indexes
uint8 STR_ERR_DI=0    # Steering module error signal
uint8 STR_BUT_DI=1    # Steering module on/off button signal
uint8 STR_WHEEL_DI=2  # Manual steering signal
uint8 THRTL_ERR_DI=3  # Throttle module error signal
uint8 THRTL_BUT_DI=4  # Throttle module on/off button signal
uint8 THRTL_PEDAL_DI=5  # Manual throttle signal
uint8 BRAKE_ERR_DI=6  # Brake module error signal
uint8 BRAKE_BUT_DI=7  # Brake module on/off button signal
uint8 BRAKE_PEDAL_DI=8  # Manual brake signal

# Relays vector indexes
uint8 BLK_LIGHTS_LEFT_REL=0  # Left blinking lights relay
uint8 BLK_LIGHTS_RIGHT_REL=1  # Right blinking lights relay
uint8 AUTO_MODE_REL=2  # Auto mode relay
uint8 ERR_LIGHT_REL=3  # Error light relay

# Header
std_msgs/Header header

# Relays status
bool[4] relays  # The current status of the 4 relays in the car interface

# Gearbox position
uint8 gear  # Position of the gearbox [0: P, 1: R, 2: N, 3: D]

# Mode
uint8 mode  # [0: Manual, 1: Auto, 2: Cooperative]

# Digital inputs status
bool[9] dig_in  # The current status of the 9 digital inputs in the car interface

# CAN communication error
bool can_comm_err  # Whether the car interface has trouble with the CAN bus communication

# Steering PID
PID str_pid