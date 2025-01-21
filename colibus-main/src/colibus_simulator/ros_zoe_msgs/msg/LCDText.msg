std_msgs/Header header

# Line of the LCD where the text is going to be shown. Can be 1, 2, or 3 (0 is reserved for current mode)
uint8 line

# The text to be shown. Avoid special characters and the length cannot be more than 20
string text
