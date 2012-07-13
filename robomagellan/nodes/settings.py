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
# minimum and maximum threshold value in HSV (hue,sat,val)
# red
#MIN_THRESH, MAX_THRESH = (163.0, 85.5, 72.5, 0.0), (189.0, 244.5, 247.5, 0.0)
# green
#MIN_THRESH, MAX_THRESH = ( 60.5, 74.5, 73.5, 0.0), (109.5, 215.5, 206.5, 0.0)
# blue
#MIN_THRESH, MAX_THRESH = ( 75.0, 80.0, 80.0, 0.0), (125.0, 230.0, 230.0, 0.0)
# yellow tennis ball
#MIN_THRESH, MAX_THRESH = (37.5, 74.0, 80.0, 0.0), (50.5, 224.0, 230.0, 0.0)
# orange cone (indoors)
MIN_THRESH, MAX_THRESH = (171.5, 106.0, 102.0, 0.0), (178.5, 248.0, 250.0, 0.0)
# 6.92 pixels == 1 cm
PIXELS_TO_CM = 6.92

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
COLLISION_DISTANCE=0.2


#
# cone capturer
#
MAX_DISTANCE_TO_CAPTURE = 2.0
SPEED_TO_CAPTURE = 0.3
SPEED_TO_ROTATE = 0.5

#
# PID Controller
#
# how close we need to get to desired yaw before moving on
THETA_TOLERANCE=.05 
# how close we need to get to desired goal before considering it reached
WAYPOINT_THRESHOLD=0.2
# feedback proportional to longitudinal (in x dir) velocity
LAMBDA=.2
# feedback proportional to lateral (in y dir) error
A1=1
# feedback proportional to angular (theta) error
A2=.65
# maximum speed for rotation
MAX_TURNRATE=1.5
# maximum speed in linear direction
MAX_VELOCITY=5.0


