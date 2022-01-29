#
# LightBarIndicator.msg
#
# @author Misheel Bayartsengel
# @version 0.1
# 
# A component interacting with the lightbar manager can request/release control and set 
# lightbar indicators by specifying its IDs in here. 

uint8 indicator

uint8 GREEN_SOLID = 0
uint8 GREEN_FLASH = 1
uint8 YELLOW_SIDES = 2
uint8 YELLOW_DIM = 3
uint8 YELLOW_FLASH = 4
uint8 YELLOW_ARROW_LEFT = 5
uint8 YELLOW_ARROW_RIGHT = 6
uint8 YELLOW_ARROW_OUT = 7