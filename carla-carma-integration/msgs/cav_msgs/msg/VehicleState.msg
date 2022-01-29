# VehicleState.msg
#
# Message defines the host vehicle state at a given point in time
#
# The fields of this message are defined relative to the vehicle state and control models described in the VehicleModelLib documentation
# This message must contain a super set of the fields
# 

#
# 2d x-axis position of the vehicle center of gravity in meters
# This position is in a fixed inertial frame which vehicle motion is described in
# 
float64 X_pos_global
#
# 2d y-axis position of the vehicle center of gravity in meters
# This position is in a fixed inertial frame which vehicle motion is described in
# 
float64 Y_pos_global
#
# The orientation of the vehicle's longitudinal axis in radians
# This orientation is in a fixed inertial frame which vehicle motion is described in
# 
float64 orientation
#
# longitudinal velocity of the vehicle center of gravity in ms in its body frame
# 
float64 longitudinal_vel
#
# lateral velocity of the vehicle center of gravity in ms in its body frame
# 
float64 lateral_vel
#
# The yaw rate of the vehicle in rads in the body frame which is equivalent to the rate of change of orientation
# 
float64 yaw_rate
# 
# The angular velocity of the front wheel in rads
# 
float64 front_wheel_rotation_rate
#
# The angular velocity of the rear wheel in rads
# 
float64 rear_wheel_rotation_rate
#
# The steering angle of the front wheel in rad with left being positive
# 
float64 steering_angle
#
# The angle the trailer makes with the vehicle longitudinal axis in rad with left being positive
# 
float64 trailer_angle
#
# The previous velocity command in ms
# 
float64 prev_vel_cmd
#
# The previous steering command in rad
# 
float64 prev_steering_cmd
