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

#oldest ::= EpochMins
uint64 oldest

# reflon ::= INTEGER (-1799999999..1800000000)
#  -- initial vertex reference longitude
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 180 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
int32 reflon

int32 REFLON_MAX = 1800000000
int32 REFLON_MIN = -1799999999
int32 REFLON_UNAVAILABLE = 1800000001


# reflat ::= INTEGER (-900000000..900000000)
#  -- initial vertex reference latitude
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
int32 reflat

int32 REFLAT_MAX = 900000000
int32 REFLAT_MIN = -900000000
int32 REFLAT_UNAVAILABLE = 900000001

#offsets ::= SEQUENCE (SIZE(3)) OF OffsetPoint 
j2735_msgs/OffsetPoint[3] offsets
