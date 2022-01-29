#
# TrafficControlGeometry.msg
#
# Part of the CARMA Cloud geo-fence specification not part of the official SAE j2735 standard
#
# J2735 2016 message format.
#
# 
# @version 0.1
#
# Description
# ...

# TrafficControlGeometry ::= SEQUENCE
# {
# 	proj IA5String (SIZE(0..63)),
# 	datum IA5String (SIZE(0..63)),
# 	reftime EpochMins,
# 	reflon Longitude,
# 	reflat Latitude,
# 	refelv Elevation, -- decimeter offset -4096 from reference datum
# 	heading INTEGER (0..3599), -- initial path heading clockwise from north in tenths of degrees
# 	nodes SEQUENCE (SIZE(1..255)) OF PathNode
# }

# proj IA5String (SIZE(0..63)),
string proj

# datum IA5String (SIZE(0..63)),
string datum

# reftime EpochMins INTEGER (0..153722867280912) -- minutes since epoch January 1, 1970 00:00 UTC
uint64 reftime

# reflon Longitude ::= INTEGER (-1799999999..1800000001)-- geographic longitude expressed in 1/10th microdegrees, 1800000001 represents null
int32 reflon

int32 REFLON_UNAVAILABLE = 1800000001
int32 REFLON_MAX = 1800000000
int32 REFLON_MIN = -1799999999

# reflat Latitude ::= INTEGER (-900000000..900000001) -- geographic latitude expressed in 1/10th microdegrees, 900000001 represents null
int32 reflat

int32 REFLAT_UNAVAILABLE = 900000001
int32 REFLAT_MAX = 900000000
int32 REFLAT_MIN = -900000000

# refelv Elevation ::= INTEGER (0..65535) -- offset by 4096, -4096 represents unknown, -409.5 to 6143.9 meters relative to referemce datum
int32 refelv

int32 REFELV_UNKNOWN = 0
int32 REFELV_MAX = 65535
int32 REFELV_MIN = 1

# heading INTEGER (0..3599) -- initial path heading clockwise from north in tenths of degrees
uint16 heading
uint16 HEADING_MAX = 3599
uint16 HEADING_MIN = 0

# nodes SEQUENCE (SIZE(1..255)) OF PathNode
j2735_msgs/PathNode[] nodes


