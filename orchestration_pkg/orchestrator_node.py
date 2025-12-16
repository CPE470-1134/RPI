# Define our Mission States
class State(Enum):
    IDLE = 0  # Waiting for input
    LOCALIZING = 1  # Spinning to find walls
    NAV_TO_P = 2  # Driving to user point P
    WAIT_AT_P = 3  # 3-second pause
    NAV_TO_GAP = 4  # Driving to EF
    CROSS_GAP = 5  # Blind/Odom traversal of opening
    FIND_TAG_1 = 6  # Search pattern outside
    DOCK_TAG_1 = 7  # Final approach
    DONE = 99
