#
# TrafficControlRequestPolygon.msg
#
# @author Misheel Bayartsengel
# @version 0.1
#
# NOTE: This message is not part of the TrafficControlRequest message definition. 
#       It is an alternative representation of TrafficControlBounds used for visualization.
#
# describes the four vertices of polygon created by CARMA to form an TrafficControlRequest zone
# each points are in EPSG:4326 lat/long                                                       

cav_msgs/Position3D[] polygon_list