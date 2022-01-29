#
# RepeatParams.msg
#
# Part of the CARMA Cloud geo-fence specification for vehicle internal use.
#
# @author Misheel Bayartsengel 
# @version 0.1
#
# Description
# ...

# RepeatParams ::= SEQUENCE
# {
# 	offset INTEGER (0..1439), -- shift repetition start relative to midnight local time
# 	period INTEGER (0..1439), -- minutes between successive time spans
# 	span INTEGER (0..1439) -- number of minutes schedule is active, must be less than the period
# }

# Offset 
# shift repetition start relative to midnight local time
uint16 offset

# Period
# minutes between successive time spans
uint16 period

# Span
# number of minutes schedule is active, must be less than the period
uint16 span
