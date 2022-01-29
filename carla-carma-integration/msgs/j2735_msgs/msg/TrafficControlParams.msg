#
# TrafficControlParams.msg
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

# TrafficControlParams ::= SEQUENCE
# {
# 	vclasses SEQUENCE (SIZE(1..255)) OF TrafficControlVehClass,
# 	schedule TrafficControlSchedule,
# 	regulatory BOOLEAN,
# 	detail TrafficControlDetail
# }

# vclasses SEQUENCE (SIZE(1..255)) OF TrafficControlVehClass,
j2735_msgs/TrafficControlVehClass[] vclasses

# schedule TrafficControlSchedule
j2735_msgs/TrafficControlSchedule schedule

# regulatory BOOLEAN
bool regulatory

# detail TrafficControlDetail
j2735_msgs/TrafficControlDetail detail

