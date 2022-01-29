#
# LightBarCDAType.msg
#
# @author Misheel Bayartsengel
# @version 0.1
# 
# A component interacting with the lightbar manager can request/release control and set 
# lightbar indicators by sending what type of Cooperative Driving Automation (CDA) msg 
# it is broadcasting. The lightbar manager would map that msg type with its corresponding
# lightbar indicator.

# Class A, Status sharing
# Class B, Intent-Sharing information
# Class C, Agreement Seeking
# Class D, Prescriptive Information

uint8 type

uint8 TypeA = 0
uint8 TypeB = 1
uint8 TypeC = 2
uint8 TypeD = 3
