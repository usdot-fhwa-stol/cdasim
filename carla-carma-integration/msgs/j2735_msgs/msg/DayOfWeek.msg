#
# DayOfWeek.msg
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

# DayOfWeek ::= BIT STRING //only J
# {
# 	sun (6), 
# 	mon (5), 
# 	tue (4), 
# 	wed (3), 
# 	thu (2), 
# 	fri (1), 
# 	sat (0)
# }

uint8[7] dow

# enumeration values for day of week
uint8 SUN = 6  
uint8 MON = 5
uint8 TUE = 4  
uint8 WED = 3  
uint8 THU = 2  
uint8 FRI = 1  
uint8 SAT = 0