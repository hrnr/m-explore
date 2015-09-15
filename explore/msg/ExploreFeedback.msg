# A goal for the exploration stack 
Header header

# Initializing node, nothing happening yet
byte INITIALIZING=0

# Exploring, everything is normal
byte EXPLORING=1

# Done exploring, there are no reachable, unknown areas left in the map
byte DONE=2

# Planner can't find plan
byte PLANNER_STUCK=3

# Controller can't find valid controls
byte CONTROLLER_STUCK=4

# Current status = {INITIALIZING, EXPLORING, DONE, PLANNER_STUCK, CONTROLLER_STUCK}
byte value
