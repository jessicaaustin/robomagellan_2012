
# define an array of ultrasonic distance sensors
# AKA the poor man's laser scanner... 
define ping laser
(
  range_max 3.0
  fov 60
  samples 60
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define rover1 position
(
  size [0.30 0.20 0.35]
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  ping(pose [ 0.090 0.000 0.185 0.000 ])
)

