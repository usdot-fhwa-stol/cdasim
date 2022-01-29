#
# TrafficControlMessageV01.msg
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

# TrafficControlMessageV01 ::= SEQUENCE
# {
# 	reqid Id64b, -- ephemeral identifier of originating request
# 	reqseq INTEGER (0..255), -- sequential counter for originating request
# 	msgtot INTEGER (0..65535), -- total expected traffic control message responses
# 	msgnum INTEGER (0..65535), -- message index for each response out of total responses
# 	id Id128b, -- unique traffic control id
# 	updated EpochMins, 
# 	package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
# 	params [1] TrafficControlParams OPTIONAL, 
# 	geometry [2] TrafficControlGeometry OPTIONAL
# }

# reqid ::= Id64b
j2735_msgs/Id64b reqid

# reqseq ::= INTEGER (0..255)
uint8 reqseq

# msgtot INTEGER (0..65535), -- total expected traffic control message responses
uint16 msgtot

# msgnum INTEGER (0..65535), -- message index for each response out of total responses
uint16 msgnum

# id Id128b, -- unique traffic control id
j2735_msgs/Id128b id

# updated EpochMins
uint64 updated

# package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
j2735_msgs/TrafficControlPackage package
bool package_exists

# params [1] TrafficControlParams OPTIONAL
j2735_msgs/TrafficControlParams params
bool params_exists

# geometry [2] TrafficControlGeometry OPTIONAL
j2735_msgs/TrafficControlGeometry geometry
bool geometry_exists
