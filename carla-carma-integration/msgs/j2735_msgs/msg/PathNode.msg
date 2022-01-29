#
# PathNode.msg
#
# Part of the CARMA Cloud geo-fence specification not part of the official SAE j2735 standard
#
# J2735 2016 message format.
#
# @author Misheel Bayartsengel
# @version 0.1
#
# The positive x direction proceeds eastward, positive y proceeds
# northward, and positive z proceeds away from the center of the Earth. 

# X ::= INTEGER (-32768..32767)
#   -- units are in cm
int16 x
int16 y
int16 z

bool z_exists

# Width ::= INTEGER (-128..127)
#   -- units are in cm
int8 width
bool width_exists