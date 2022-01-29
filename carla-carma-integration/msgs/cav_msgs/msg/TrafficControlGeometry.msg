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

# reftime EpochMins,
time reftime

# reflon Longitude,
float64 reflon

# reflat Latitude,
float64 reflat

# refelv Elevation, -- decimeter offset -4096 from reference datum
float32 refelv

# heading INTEGER (0..3599), -- initial path heading clockwise from north in tenths of degrees
float32 heading

# nodes SEQUENCE (SIZE(1..255)) OF PathNode
cav_msgs/PathNode[] nodes


