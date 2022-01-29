# LaneChangeManeuver.msg
#
# Message defining a Lane Change Maneuver for use in CARMA planning. 
# This message should only be used to describe lane changes to adjacent lanes
# Lane changes that cross multiple lanes should be described using a sequence of maneuvers
#

# The common parameters of this maneuver
cav_msgs/ManeuverParameters parameters

# Approximate starting state of the maneuver
float64 start_dist # downtrack distance in m
float64 start_speed # vehicle speed in m/s
time start_time

# Approximate ending state of the maneuver
float64 end_dist 
float64 end_speed
time end_time

# GUID of the starting lane of this maneuver
string starting_lane_id
# GUID of the ending lane of this maneuver
string ending_lane_id
