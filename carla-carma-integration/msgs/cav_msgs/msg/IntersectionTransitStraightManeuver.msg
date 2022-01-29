# IntersectionTransitStraightManeuver.msg
#
# Message defining a Intersection Transit Straight Maneuver
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
