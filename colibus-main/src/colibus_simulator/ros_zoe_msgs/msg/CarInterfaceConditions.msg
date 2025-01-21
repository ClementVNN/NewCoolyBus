# Switch to manual conditions vector indexes
uint8 MAN_STR=0        # Manual steering condition
uint8 MAN_BRAKE=1      # Manual brake condition
uint8 MAN_THTTL=2      # Manual throttle condition
uint8 GEAR_PARKING=3   # Gear in parking condition
uint8 SET_MANUAL=4     # Set manual mode condition
uint8 MODE_MANUAL=5    # Manual mode condition
uint8 STR_MOD_ERR=6    # Steering module error condition
uint8 BRK_MOD_ERR=7    # Brake module error condition
uint8 TTHL_MOD_ERR=8   # Throttle module error condition

# Car stop conditions vector indexes
uint8 DOOR_OPEN=0      # Door open condition
uint8 HAND_BRAKE=1     # Hand brake engaged condition
uint8 SEAT_BELT=2      # Seat belt not fastened condition
uint8 CAN_ERR=3        # CAN bus error condition
uint8 UART_ERR=4       # UART communication error condition
uint8 PC_ERR=5         # PC error conditionHeader header

# Status of the conditions vector to switch to manual the vehicle
bool[9] switch_to_man

# Status of the conditions vector to stop the vehicle
bool[6] cond_stop
