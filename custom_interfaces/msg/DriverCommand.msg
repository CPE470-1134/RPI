# Command from controller to driver for simple motion primitives

# Command types
uint8 STOP = 0
uint8 ROTATE = 1
uint8 DRIVE = 2

# Command type (use constants above)
uint8 command_type

# Target value:
# - For ROTATE: target angle in radians (relative rotation)
# - For DRIVE: target distance in meters (forward positive)
# - For STOP: ignored
float32 target_value

# Speed:
# - For ROTATE: angular speed in rad/s
# - For DRIVE: linear speed in m/s
# - For STOP: ignored
float32 speed
