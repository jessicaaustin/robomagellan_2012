# constants for ROS project
#
# all parameters are in SI units (meters, m/s, etc)
#
# TODO: the "right way" to do this is via a ROS parameter server
#       http://www.ros.org/wiki/rospy/Overview/Parameter%20Server

#
# color tracker constants
#

# the index of your camera. if /dev/videoN is your camera device, then MY_CAMERA=N
MY_CAMERA = 0
# how many previous positions to interpolate to find our current position. higher smoothness => slower tracking, but less jerkiness 
SMOOTHNESS = 4
# The maximum and minimum values for cone coordinate
CONE_MAX_Y=0.20
CONE_MAX_X=0.18
# minimum and maximum threshold value in HSV (hue,sat,val)
# red
#MIN_THRESH, MAX_THRESH = (163.0, 85.5, 72.5, 0.0), (189.0, 244.5, 247.5, 0.0)
# green
#MIN_THRESH, MAX_THRESH = ( 60.5, 74.5, 73.5, 0.0), (109.5, 215.5, 206.5, 0.0)
# blue
#MIN_THRESH, MAX_THRESH = ( 75.0, 80.0, 80.0, 0.0), (125.0, 230.0, 230.0, 0.0)
# yellow tennis ball
#MIN_THRESH, MAX_THRESH = (37.5, 74.0, 80.0, 0.0), (50.5, 224.0, 230.0, 0.0)
# orange cone 
MIN_THRESH, MAX_THRESH = (0.0, 140.0, 160.0, 0.0), (3.0, 250.0, 270.0, 0.0)
# red bin (backyard)
#MIN_THRESH, MAX_THRESH = (160.0, 100.0, 100.0, 0.0), (170.0, 250.0, 250.0, 0.0)

# 6.92 pixels == 1 cm
PIXELS_TO_CM = 6.92

#
# motors
#
# the percentage of maximum motor rotational speed, at which
# the motors should run, in order to rotate the rover.
rotationWheelSpeed = 25.0
# the percentage of maximum motor rotational speed, at which
# the motors should run, in order to translate the rover
translationWheelSpeed = 70.0

#
# IMU module
#
minYawThreshold = 0.03
maxYawThreshold = 0.78

#
# GPS module
# TODO: figure out LAT_OFFSET and LNG_OFFSET
#
KNOT_TO_M_S = 0.514444444
UTM_ZONE=34
LAT_OFFSET = 0.0056
LNG_OFFSET = -0.2683


#
# collision detector
#
COLLISION_DISTANCE=0.3      # how close we have to get to something (e.g. a cone) to consider "hitting" it
OBSTACLE_DISTANCE=0.6       # how close something should be to consider it an obstacle

#
# Navigation
#
                # 15 degrees
THETA_TOLERANCE=0.26                  # how close we need to get to desired yaw before moving on
WAYPOINT_THRESHOLD=0.5                # how close we need to get to desired goal before considering it reached
DISTANCE_TO_CAPTURE = 2.0             # how close we should be to a cone before we attempt to capture it
                                      #  if we also have visual confirmation of the cone
DISTANCE_TO_CAPTURE_NO_VISUAL = 1.0   # how close we should be to a cone before we attempt to capture it
                                      #  if we do NOT have visual confirmation of the cone
CONE_CAPTURE_COLLISION_DISTANCE=0.2   # how close we need to get to the cone to consider it "captured"

#
# Rover contraints
#

MAX_TURNRATE=2.0        # maximum speed for rotation
MIN_TURNRATE=2.0        # minimum speed for in-place rotation
MAX_VELOCITY=0.55       # maximum speed in linear direction
MIN_VELOCITY=0.30       # minimum speed in linear direction

#
# PID Controller
#

# Moving in a straight path towards a goal:
KP_X=2.0                      # feedback proportional to longitudinal (in x dir) velocity
KP_Y=0.1                     # feedback proportional to lateral (in y dir) error
KI_Y=0.00                     # feedback for integral of lateral error
YERR_ACCUMULATED_MAX=1.57     # max amount of accumulated integral error (absolute value)
KD_Y=0.00                     # feedback for derivative of lateral error

# Rotating in place: 
KP_T=0.5    # feedback proportional to angular (theta) error
ROTATE_CYCLES=2       # How many cycles
ROTATE_VEL=2.0        # m/s

# Capturing cone
KP_CT=1    # feedback proproprtional to y-err 

# Rotating to find cone
CONE_SEARCH_ROT_TIME=4       # How many cycles
CONE_SEARCH_ROT_VEL=2.0       # m/s
CONE_CAPTURE_CYCLE_TIME=8    # How many cycles to move forward before pausing

