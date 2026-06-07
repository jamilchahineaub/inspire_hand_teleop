# Republished Inspire hand state from the Unitree DDS topic rt/inspire/state.
# Per-hand joint order matches the Unitree example:
#   [pinky, ring, middle, index, thumb_bend, thumb_rotation]
# q values are normalized: 0 = close, 1 = open.
std_msgs/Header header
float32[6] right_q
float32[6] left_q
