# LightBarStatus.msg

# This message contains the state of the light bar 
# the states are set based off of front lightbar

uint8 green_solid
uint8 yellow_solid
uint8 right_arrow
uint8 left_arrow
uint8 sides_solid
uint8 flash
uint8 green_flash

uint8 OFF = 0
uint8 ON = 1

# following state(s) may not be supported anymore
uint8 takedown


