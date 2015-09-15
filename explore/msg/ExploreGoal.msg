# A goal for the exploration stack 
Header header

# Don't explore (could listen for goal instead?)
byte NONE=0

# Explore frontiers
byte FRONTIER=1

# Explore based on entropy calculations
byte ENTROPY=2

# Current goal = {NONE, FRONTIER, ENTROPY}
byte value