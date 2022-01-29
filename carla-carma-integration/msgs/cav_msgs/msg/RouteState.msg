#
# RouteState.msg
# 
# State of route progress including orthogonal distance from 
# vehicle to current lanelet center line, parametric distance along
# route, completion status, and the length of the route 
# provided in various metrics
#
# @author Michael McConnell
# @version 0.1
#

Header header

# The route id for which this message applies
string routeID

# Enum denoting the state of the route following
uint8 state

uint8 LOADING=0
uint8 SELECTION=1
uint8 ROUTING=2
uint8 FOLLOWING=3

# Cross-track distance of vehicle along route in meters
float64 cross_track

# Down-track distance of vehicle along route in meters
float64 down_track

# Down-track distance of vehicle along the current lanelet in meters
float64 lanelet_downtrack

# Current host vehicle lane index
int64 lanelet_id

# Current speed limit on current lanelet in m/s
float64 speed_limit
