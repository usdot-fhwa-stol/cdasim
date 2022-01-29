# LaneFollowingManeuver.msg
#
# Message defining a Lane Following Maneuver for use in CARMA planning. 
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

# List of lanes this maneuver will cover. They should all be contigous lanes connected end to end (ie. no lane changes)
string[] lane_ids
