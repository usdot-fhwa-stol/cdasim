# LocalizationStatusReport.msg

# This message reports the current status of CARMA localization.

# Standard ROS header
std_msgs/Header header

# Enum describing the current status of localization
uint8 status

uint8 UNINITIALIZED=0 # Entry state of localization system before any intialization has occured. Should only occur at startup.
uint8 INITIALIZING=1 # State where system is currently initializing.
uint8 OPERATIONAL=2 # State representing that the system is initialized and localization has a good fitness score and publication rate.
uint8 DEGRADED=3 # State representing that the system is initialized, but has a poor fitness score or publication rate.
uint8 DEGRADED_NO_LIDAR_FIX=4 # State representing localization is executing without lidar based localization data. 
uint8 AWAIT_MANUAL_INITIALIZATION=5 # State representing that the system was initialized but localization data can no longer be relied upon. 