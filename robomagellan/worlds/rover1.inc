
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
  size [0.65 0.65 0.25]
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  ping(pose [ 0.275 0.000 0 0.000 ])
)

