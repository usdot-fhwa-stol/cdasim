#
# Trailer Angle Message
# 
# This message gives the truck trailer angle in radians
#
# @author Aswath Suresh
# @version 0.1
#

std_msgs/Header header  # timestamp in the header is the time the sensor returned the angle reading

float64 angle #trailer angle in raidans. Note if the sensor 1 is mounted on the left and sensor 2 is mounted on the right. Then the left turn will produce negative radians whereas the right turn will produce positive radians. If sensor placement are swapped, the signs become opposite to previous setup.

