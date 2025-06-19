std_msgs/Header header

# Current state of the virtuose node
# 0: Unknown
# 1: Ready
# 2: Error
# 3: Calibration
# 4: Cart_Imp
# 5: Cart_Adm
uint8 state

# Emergency stop
# True is emergency stop occurred
bool emergency_stop

# Motor power supply
# True is 48V is ON
bool power_supply

# Force-feedback button
# True if button is PUSHED
bool power_button

# Other buttons
# Bitfield
uint32 buttons

