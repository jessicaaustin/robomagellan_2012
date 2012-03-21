#
# Defines a traffic cone 
#

define cone model
(
  bitmap "circle.png"
  # 18 inch traffic cone
  size [0.3 0.3 0.45]
  color "orange"
  gui_outline 0
  # allows the cone to be pushed about
  gripper_return 1
  obstacle_return 0
)
