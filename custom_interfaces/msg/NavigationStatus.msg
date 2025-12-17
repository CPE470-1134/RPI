# Navigation status from controller

# Status phases
uint8 IDLE = 0
uint8 ALIGNING = 1
uint8 MOVING = 2
uint8 FINAL_ROTATION = 3
uint8 APPROACHING_MARKER = 4
uint8 DONE = 5

# Current phase
uint8 phase

# Progress (0.0 to 1.0)
float32 progress

# Distance remaining to target (meters)
float32 distance_remaining

# Human-readable status message
string message
