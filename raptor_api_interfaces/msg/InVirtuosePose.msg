std_msgs/Header header

# Must be equal to the ID given by virtuose_admittance
uint32 client_id

# Position and orientation of the Virtuose control point (wrist)
# Expressed in the World reference frame
geometry_msgs/Transform virtuose_pose

