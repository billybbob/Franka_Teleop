std_msgs/Header header

# Must be equal to the ID given by virtuose_impedance
uint32 client_id

# Force applied by the user at the Virtuose control point (wrist)
# Expressed as a wrench in the World reference frame
geometry_msgs/Wrench virtuose_force
