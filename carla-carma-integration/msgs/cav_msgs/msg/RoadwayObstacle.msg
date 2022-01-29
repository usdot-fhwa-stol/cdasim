#
# RoadwayObstacle.msg
# 
# The frame is a FRD frame located on the uptrack waypoint of a segment. 
# With +x pointing along the segment and +y pointing to the right
#
# @author Michael McConnell
# @version 0.1
#

# The geometry of the vehicle
cav_msgs/ExternalObject object

# Enum describing the type of communication this vehicle is capable of
cav_msgs/ConnectedVehicleType connected_vehicle_type

# id of the lanelet this obstacle occupies
int64 lanelet_id

# Cross-track distance of obstacle along lanelet
float64 cross_track

# Down-track distance of obstacle along lanelet
float64 down_track

# predicted id of the lanelets this obstacle will occupy
int64[] predicted_lanelet_ids

# Prediction Confidence [0,1]
float64[] predicted_lanelet_id_confidences

# Predicted Cross-track distance of obstacle along lanelets
float64[] predicted_cross_tracks

# Prediction Confidence [0,1]
float64[] predicted_cross_track_confidences

# Predicted Down-track distance of obstacle along lanelets
float64[] predicted_down_tracks

# Prediction Confidence [0,1]
float64[] predicted_down_track_confidences

