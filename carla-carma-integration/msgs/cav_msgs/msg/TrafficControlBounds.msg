#
# TrafficControlBounds.msg
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


#TrafficControlBounds ::= SEQUENCE
#{
#	oldest EpochMins, -- timestamp of oldest traffic control message within bounds
#	reflon Longitude, -- initial vertex reference longitude
#	reflat Latitude, -- initial vertex reference latitude
#	offsets SEQUENCE (SIZE(3)) OF OffsetPoint -- relative vertex positions in scaled meters
#}

# Oldest timestamp in seconds
#  -- timestamp of oldest control message within bounds
time oldest

# Longitude in degrees with a range of (-179.9999999, 180.0)
# -- initial vertex reference longitude
float64 reflon

# Latitude in degrees with a range of (-90.0, 90.0)
# -- initial vertex reference latitude
float64 reflat

# Offsets
#  -- relative vertex positions in meters
cav_msgs/OffsetPoint[3] offsets

