/* Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
*/

#include<CARMANodeHandle.h>
#include<carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/VehicleParameters.h>
#include<carla/client/Vehicle.h>
#include<carla/Unreal/CarlaUE4/Source/CarlaUE4/CarlaUE4.h>
#include<Carla/Actor/VehicleParameters.h>



namespace vehicle{

class VirtualVehicle public Actor
{
    public:

        /**
         * @brief Constructor
        */
        VirtualVehicle();

        /**
         * @brief Initialize the virtual vehicle using the parameters from carma-config/carla_integration
        */
        void initialize();

    private:
        ros::CARMANodeHandle pnh;
        bool use_sim_time_;
        //Host vehicle make
        std::string vehicle_make_;

        //Host vehicle model
        std::string vehicle_model_;

        //Host vehicle year
        int vehicle_year_;

        //Host vehicle length
        double vehicle_length_;

        //Host vehicle width
        double vehicle_width_;

        //Host vehicle height
        double vehicle_height_;

        //Distance from front axel to rear axel
        double vehicle_wheel_base_;

        //Radius of the tires
        double vehicle_tire_radius_;

        //Acceleration limit
        double vehicle_acceleration_limit_;

        //Deceleration limit
        double vehicle_deceleration_limit_;

        //Lateral Acceleration Limit
        double vehicle_lateral_accel_limit_;

        //Lateral Jerk Limit
        double vehicle_lateral_jerk_limit_;

        //Max curvature rate
        double vehicle_max_curvature_rate_;

        //Ratio relating the steering wheel angle and the tire position on the road (radians of a full steering wheel rotation) / (radians of tires with the longitudinal axis under full steer)
        double vehicle_steering_gear_ratio_;

        //Maximum steering angle of the wheel relative to the vehicle centerline.
        double vehicle_steer_lim_deg_;

        //steering dynamics time constant
        double vehicle_model_steer_tau_;

        carla::client::Vehicle carla_vehicle_;
        carla::client::Actor veh;
        
        
            




}//end class

}