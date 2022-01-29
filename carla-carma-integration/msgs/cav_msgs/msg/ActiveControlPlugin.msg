# ActiveControlPlugin.msg

# This message describes the name of a control plugin that is currently control the host vehicle
# and its trajectory execution status.

# Standard ROS header
std_msgs/Header header

# Name of the control plugin which is actively controlling the host vehicle
string plugin_name

# GUID of the current following trajectory
string  current_trajectory_id
