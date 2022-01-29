# StopAndWaitManeuver.msg
#
# Message defining a Stop and Wait Maneuver for use in CARMA planning. 
#

# The common parameters of this maneuver
cav_msgs/ManeuverParameters parameters

# Approximate starting state of the maneuver
float64 start_dist # downtrack distance in m
float64 start_speed # vehicle speed in m/s
time start_time

# Approximate ending state of the maneuver
float64 end_dist
time end_time
# The end speed is always zero

# GUID of the starting lane of this maneuver
string starting_lane_id
# GUID of the ending lane of this maneuver
string ending_lane_id
