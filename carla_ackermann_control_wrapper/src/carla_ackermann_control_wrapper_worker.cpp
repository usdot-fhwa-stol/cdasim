/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "carla_ackermann_control_wrapper_worker.h"

namespace carla_ackermann_control_wrapper {

     double CarlaAckermannControlWrapperWorker::convert_trans_rot_vel_to_steering_angle(const float speed, const float omega, const double wheel_base){
        if (speed == 0 || omega == 0) {
            return 0;
        } else {
            float radius = speed / omega;
            return atan(wheel_base / radius);
        }
    }      

    // convert the Twist message from Autoware to an AckermannDrive message
    cav_msgs::AckermannDrive CarlaAckermannControlWrapperWorker::update_ackermanndrive_cmd(const autoware_msgs::VehicleCmd& msg, const double wheel_base){
        ackermann_drive_.speed = msg.twist_cmd.twist.linear.x;
        ackermann_drive_.steering_angle = convert_trans_rot_vel_to_steering_angle(msg.twist_cmd.twist.linear.x, msg.twist_cmd.twist.angular.z, wheel_base);
        return ackermann_drive_;
    }
}
