#
# OffsetPoint.msg
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

#OffsetPoint ::= SEQUENCE
#{
#	deltax INTEGER (-32768..32767), -- relative horizontal vertex position in scaled meters
#	deltay INTEGER (-32768..32767) -- relative vertical vertex position in scaled meters
#}

#deltax ::= INTEGER (-32768..32767)
float64 deltax

#deltay ::= INTEGER (-32768..32767)
float64 deltay


