#include<virtual_vehicle.h>

namespace vehicle{

    VirtualVehicle::VirtualVehicle()
    {
        /*Add a CARLA Actor for the virtual vehicle*/
    }

    void VirtualVehicle::initialize()
    {
        /*Acquire the parameters from carma-config*/
        pnh.param<bool>('/use_sim_time',use_sim_time_,true);
        pnh.param<std::string>('/vehicle_make', vehicle_make_,"Default");
        pnh.param<std::string>('/vehicle_model', vehicle_model_, "Default_Model");
        pnh.param<int>('/vehicle_year',vehicle_year_, 1999);
        pnh.param<double>('/vehicle_length', vehicle_length_, 1.0);
        pnh.param<double>('/vehicle_width', vehicle_width_, 2.0);
        pnh.param<double>('/vehicle_height', vehicle_height_, 3.0);
        pnh.param<double>('/vehicle_wheel_base', vehicle_wheel_base_, 4.0);
        pnh.param<double>('/vehicle_tire_radius',vehicle_tire_radius_, 1.0);
        pnh.param<double>('/vehicle_acceleration_limit', vehicle_acceleration_limit_, 1.0);
        pnh.param<double>('/vehicle_deceleration_limit', vehicle_deceleration_limit_, 1.0);
        pnh.param<double>('/vehicle_lateral_accel_limit', vehicle_lateral_accel_limit_, 2.0);
        pnh.param<double>('/vehicle_lateral_jerk_limit',vehicle_lateral_jerk_limit_, 1.0);
        pnh.param<double>('/vehicle_max_curvature_rate', vehicle_max_curvature_rate_,1.0);
        pnh.param<double>('/vehicle_steering_gear_ratio', vehicle_steering_gear_ratio_,1.2);
        pnh.param<double>('/vehicle_steer_lim_deg',vehicle_steer_lim_deg_, 2.1);
        pnh.param<double>('/vehicle_model_steer_tau', vehicle_model_steer_tau_, 0.9);

        /*Assign the newly acquired parameters to the virtual-vehicle as attributes*/
        carla::client::ActorAttribute vehicle_make;
        




    }
}