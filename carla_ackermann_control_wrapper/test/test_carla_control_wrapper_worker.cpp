/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <gtest/gtest.h>

namespace carla_ackermann_control_wrapper
{
    Test(CarlaAckermannControlWrapperWorkerTest, testAckermannDriveConversion)
    {
        CarlaAckermannControlWrapperWorker worker;

        double wheel_base = 3.0;

        autoware_msgs::VehicleCmd vehicle_cmd;
        vehicle_cmd.header.stamp = ros::Time::now();
        vehicle_cmd.twist_cmd.twist.linear.x = 5.0;
        vehicle_cmd.twist_cmd.twist.angular.z = 0.785;
        vehicle_cmd.ctrl_cmd.linear_velocity = 4.0;
        vehicle_cmd.ctrl_cmd.linear_acceleration = 3.0;
        vehicle_cmd.ctrl_cmd.steering_angle = 0.393;
        vehicle_cmd.accel_cmd.accel = 10;
        vehicle_cmd.brake_cmd.brake = 20;
        vehicle_cmd.steer_cmd.steer = 30;
        vehicle_cmd.lamp_cmd.l = 1;
        vehicle_cmd.lamp_cmd.r = 1;
        vehicle_cmd.mode = 6;
        vehicle_cmd.emergency = 0;

        EXPECT_EQ(0.44 , worker.update_ackermanndrive_cmd(vehicle_cmd, wheel_base).steering_angle, 0.001);
    }
}