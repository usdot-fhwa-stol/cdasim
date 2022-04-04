#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
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
#
#

"""
Class for testing virtual vehicle
"""

import unittest
import rospy
import rostest
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, NavSatFix, Image, PointCloud2, Imu
from geometry_msgs.msg import Quaternion, Vector3, Pose
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
from visualization_msgs.msg import Marker
from carla_msgs.msg import (CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaWorldInfo)


PKG = 'test_roslaunch'


class TestNode(unittest.TestCase):
    #Insert test cases
    print('0')

def test_vehicle_status(self):
        """
        Tests vehicle_status
        """
        rospy.init_node('test_node', anonymous=True)
        #msg = rospy.wait_for_message(
        #    "/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, timeout=TIMEOUT)
        #self.assertNotEqual(msg.header, Header())  # todo: check frame-id
        #self.assertNotEqual(msg.orientation, Quaternion())


if __name__ == '__main__':
    rostest.rosrun(PKG, 'tests', TestNode)
