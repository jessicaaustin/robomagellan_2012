Header header
uint8 state
uint8 NONE                    = 0     # Uninitialized
uint8 ROTATE_INTO_POSITION    = 1     # Initially, rotate the robot to face towards the goal
uint8 MOVE_TOWARDS_GOAL       = 2     # Moving in a straight path towards the goal
uint8 CAPTURE_CONE            = 3     # Using the cone_coord data to capture the cone
uint8 AVOID_OBSTACLE          = 4     # Navigating around an obstacle
uint8 ROTATE_TO_FIND_PATH     = 5     # Can't find a clear path to goal, rotate in place
string text     # a human-friendly version of the state
