#
# Plugin Message
# 
# The details of a discovered plugin
#
# @author Kyle Rush
# @version 0.1
#

# The string name identifier for the plugin
string name

# The string version identifier for the plugin
string versionId

# The usage type for the plugin
uint8 type
uint8 UNKNOWN = 0
uint8 STRATEGIC = 1
uint8 TACTICAL = 2
uint8 CONTROL = 3

# Whether or not the plugin is currently available to generate a trajectory
# Availability should be determined by plugin itself
bool available

# Whether or not the plugin is activated by user
bool activated

# Indicate plugin's capabilitiy in a hierarchy structure: "capbilitiy_A/feature_B_of_capbilitiy_A/implementation_C_of_feature_B"
string capability