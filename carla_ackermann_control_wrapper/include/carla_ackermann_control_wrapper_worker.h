#pragma once

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
#include <math.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <autoware_msgs/VehicleCmd.h>

namespace carla_ackermann_control_wrapper {

class CarlaAckermannControlWrapperWorker {

    public:

        // convert the Twist message from Autoware to an AckermannDrive message
        ackermann_msgs::AckermannDrive update_ackermanndrive_cmd(const autoware_msgs::VehicleCmd& msg, const double wheel_base);

    private:

        double convert_trans_rot_vel_to_steering_angle(const float speed, const float omega, const double wheel_base);

        // local variables
        autoware_msgs::VehicleCmd vehicle_cmd_;
        ackermann_msgs::AckermannDrive ackermann_drive_;
};
}