# PluginStatus.msg
#
# Announces the status and category of plugin publishing this message 
#

# Plugin name is the ROS name of the plugin node
string  name

# status indicates the operational state of the plugin,
# and is an enumerated value.
uint8  status
# enumeration values for status:
uint8  DISABLED=0
uint8  ENABLED=1
uint8  AVAILABLE=2
uint8  FAULT=3
#					<other values reserved for future expansion>

# Possible plugin catagories
bool is_strategic_plugin
bool is_tactical_plugin
bool is_control_plugin
