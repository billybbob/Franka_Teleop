std_msgs/Header header

# Must be equal to the ID given by virtuose_admittance
uint32 client_id

# Speed of the Virtuose control point (wrist)
# Expressed as a twist in the World reference frame
geometry_msgs/Twist virtuose_speed
