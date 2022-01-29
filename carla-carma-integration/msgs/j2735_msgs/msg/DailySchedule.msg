#
# DailySchedule.msg
#
# Part of the CARMA Cloud geo-fence specification for vehicle internal use.
#
# @author Misheel Bayartsengel 
# @version 0.1
#
# Description
# The schedule bounds of a geofence within a specific day of the week

# DailySchedule ::= SEQUENCE
# {
# 	begin INTEGER (0..1439), -- minutes relative to midnight local time
# 	duration INTEGER (0..1439) -- the number of active minutes, up to one day
# }

# begin INTEGER (0..1439), -- minutes relative to midnight local time
uint16 begin

# duration INTEGER (0..1439) -- the number of active minutes, up to one day
uint16 duration