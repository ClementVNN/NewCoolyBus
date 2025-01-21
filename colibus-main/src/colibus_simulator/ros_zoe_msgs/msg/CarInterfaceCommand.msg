std_msgs/Header header

uint8 hd          # Header byte
uint8 cmd         # Command byte
uint8 len         # Data length byte
uint8[] data      # Data (an array of bytes)
uint16 chsum      # Checksum (16-bit value)
