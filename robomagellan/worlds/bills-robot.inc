# 
# Stage definition for Bill's robot from 2011 competition
#

# IR sensor: Parallx PING))) 
define parallaxping laser
(
  # 2cm to 3m range
  range_min 0.02
  range_max 3.0

  # field of view
  # (says 40 deg on datasheet but this has not been verified)
  # TODO verify fov for laser
  fov 20.0

  # number of laser samples per scan
  samples 10

  # how to display on the robot model
  color "blue"
  size [ 0.02 0.05 0.03 ]
)

# The model for our robot
# mass: 6.3 kg
define robot position
(
  # x y z
  size [0.64 0.54 0.54]
  # wheels are offset forward from the center
  origin [-0.11 0 0 0]
  gui_nose 1

  drive "diff"

  # Can hit things
  obstacle_return 1

  # TODO add other sensors
  parallaxping(pose [ 0.220 0.085 -0.390 0.000 ])
)


