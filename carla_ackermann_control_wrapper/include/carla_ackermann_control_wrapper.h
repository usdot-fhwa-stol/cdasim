#pragma once

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

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/DriverStatus.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/CarlaEnabled.h>
#include <cav_msgs/AckermannDrive.h>
#include <cav_msgs/CarlaEgoVehicleControl.h>
#include <cav_msgs/CarlaEgoVehicleInfo.h>
#include <cav_msgs/CarlaEgoVehicleStatus.h>
#include <cav_msgs/CarlaEgoVehicleInfoWheel.h>
#include <cav_srvs/SetEnableRobotic.h>
#include <autoware_msgs/VehicleCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "carla_ackermann_control_wrapper_worker.h"

namespace carla_ackermann_control_wrapper {

class CarlaAckermannControlWrapper
{
    public:

        CarlaAckermannControlWrapper();

        /*!
         * \brief Begin the execution of Carla Ackermann COntrol Wrapper node.
         * 
         * \return The exit status of this program
         */
        int run();

    private:
        // node handles
        ros::CARMANodeHandle nh_, pnh_;

        // topic subscribers
        ros::Subscriber vehicle_cmd_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;
        ros::Subscriber carla_enabled_sub_;

        // topic publishers
        ros::Publisher ackermanndrive_pub_;
        ros::Publisher robot_status_pub_;
        ros::Publisher vehicle_info_pub_;
        ros::Publisher vehicle_status_pub_;
        ros::Publisher driver_status_pub_;
        
        // delegate logic implementation to worker class
        CarlaAckermannControlWrapperWorker worker_;

        // initialize method
        void init();

        // message/service callbacks
        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);
        void vehicle_cmd_cb(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd);
        void robot_status_cb(const cav_msgs::CarlaEnabledConstPtr& msg);

        // check controller health status
        void update_controller_health_status();

        // local variables
        autoware_msgs::VehicleCmd vehicle_cmd_;
        cav_msgs::AckermannDrive ackermann_drive_;
        cav_msgs::CarlaEnabled carla_status_;
        cav_msgs::RobotEnabled robotic_status_;
        cav_msgs::DriverStatus driver_status_;
        cav_msgs::CarlaEgoVehicleInfo ego_info_;
        cav_msgs::CarlaEgoVehicleStatus ego_status_;
        boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;

        double wheel_base_;
        double max_steer_angle_;
        double max_speed_;
        double vehicle_mass_;
        double max_accel_;
        double max_decel_;
        double current_speed_;
};
}