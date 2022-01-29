#
# TrafficControlRequestV01.msg
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

#TrafficControlRequestV01 ::= SEQUENCE
#{
#	reqid Id64b, -- randomly generated ephemeral request identifier
#	reqseq INTEGER (0..255), -- sequential counter used to differentiate traffic control requests
#	scale INTEGER (-3..3), -- vertex offset units 10^n meters
#	bounds SEQUENCE (SIZE(1..63)) OF TrafficControlBounds
#}


# reqid ::= Id64b
j2735_msgs/Id64b reqid

# reqseq ::= INTEGER (0..255)
uint8 reqseq

# Scale ::= INTEGER (-3..3)  
int8 scale

# Bounds SEQUENCE (SIZE(1..63)) OF TrafficControlBounds
j2735_msgs/TrafficControlBounds[] bounds

































