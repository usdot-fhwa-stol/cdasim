# ManeuverParameters.msg
#
# Message defining shared parameters of Maneuvers in the CARMA platform
#

# The unique GUID of this maneuver
# The maneuver ID assigns a unique ID to the physical maneuver being performed not the ros message or code object
# Therefore maneuver ID is independant of the message and can persit across maneuver plans
string  maneuver_id

# Enum indicating the type of negotiation this maneuver is involved in
uint8 negotiation_type

uint8 NO_NEGOTIATION = 0
uint8 GENERAL_NEGOTIATION = 1
uint8 PLATOONING = 2
# Remaining enum values reserved for future use

# The strategic plugin which planned this maneuver
string planning_strategic_plugin

# Optional Fields

# Maneuver parameters presense vector

uint16 presence_vector

uint16 HAS_TACTICAL_PLUGIN = 1
uint16 HAS_BOOL_META_DATA = 2
uint16 HAS_INT_META_DATA = 4
uint16 HAS_FLOAT_META_DATA = 8
uint16 HAS_STRING_META_DATA = 16
uint16 HAS_TIME_META_DATA = 32

# The tactical plugin which planned this maneuver
string planning_tactical_plugin

# Optional meta data arrays which can be used to store additional information in the maneuver
bool[] bool_valued_meta_data
int64[] int_valued_meta_data
float64[] float_valued_meta_data
string[] string_valued_meta_data
time[] time_valued_meta_data
