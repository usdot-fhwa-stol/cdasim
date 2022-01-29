# ManeuverPlan.msg
#
# Message defining a set of maneuvers which are meant to be executed in sequence
#

# Header
std_msgs/Header header

# The unique GUID of this maneuver plan
string  maneuver_plan_id

# The time planning for this maneuver plan began
time planning_start_time

# The time planning for this maneuver completed
time planning_completion_time

# The set of maneuvers defining this plan
cav_msgs/Maneuver[] maneuvers
