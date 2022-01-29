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

# Units are in m in all of these
float64 x
float64 y
float64 z

bool z_exists

# Width: units in m
float32 width
bool width_exists
    