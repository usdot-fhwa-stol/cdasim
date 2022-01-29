#
# UpcomingLaneChangeStatus.msg
#

# downtrack until next lanechange
float64 downtrack_until_lanechange

# choice for lane change
uint8 lane_change

# enums for lane change
uint8 NONE = 0

uint8 LEFT = 1

uint8 RIGHT= 2
