#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive autoware_msgs::VehicleCmd and publish ackermann_msgs::AckermannDrive
"""
import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from autoware_msgs.msg import VehicleCmd
from std_msgs.msg import Bool

pub = None
wheelbase = 2.7


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    """
    convert translation and rotation velocity to steering angle
    """
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    return math.atan(wheelbase / radius)


def callback(data):
    """
    callback for vehicle cmds
    """
    msg = AckermannDrive()
    msg.speed = data.twist_cmd.twist.linear.x
    msg.steering_angle = convert_trans_rot_vel_to_steering_angle(msg.speed,
                                                                 data.twist_cmd.twist.angular.z,
                                                                 wheelbase)
    pub.publish(msg)


def twist_to_ackermanndrive():
    """
    mainloop
    """
    global pub, wheelbase
    rospy.init_node('twist_to_ackermanndrive')
    wheelbase = rospy.get_param('~wheelbase', 2.7)
    role_name = rospy.get_param('/role_name', 'ego_vehicle')
    pub = rospy.Publisher('/carla/{}/ackermann_cmd'.format(role_name), AckermannDrive, queue_size=1)
    rospy.Subscriber('/vehicle_cmd', VehicleCmd, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    twist_to_ackermanndrive()
