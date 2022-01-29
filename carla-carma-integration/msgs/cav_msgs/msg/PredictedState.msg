#
# PredictedState.msg
# 
# Describes the predicted states of non-stationary external objects over one time step.

# Header contains the timestamp and frame id which the remaining fields reference.
Header header

# Predicted position of the object
geometry_msgs/Pose predicted_position

# Prediction confidence [0,1]
float64 predicted_position_confidence

# Predicted velocity of the object
geometry_msgs/Twist predicted_velocity

# Prediction confidence [0,1]
float64 predicted_velocity_confidence