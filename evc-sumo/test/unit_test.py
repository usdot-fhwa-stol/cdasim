 #   Copyright (C) 2022 LEIDOS.
 #
 #   Licensed under the Apache License, Version 2.0 (the "License"); you may not
 #   use this file except in compliance with the License. You may obtain a copy of
 #   the License at
 #
 #   http://www.apache.org/licenses/LICENSE-2.0
 #
 #   Unless required by applicable law or agreed to in writing, software
 #   distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 #   WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 #   License for the specific language governing permissions and limitations under
 #   the License.
 #


import unittest
import os
import sys
from pathlib import Path

sys.path.append('../evc_sumo_bridge')
from evc_connector import EvcConnector

class TestControllerIOGreen(object):
    def is_cob_on(self, index):
        green_list = [i for i in range(0, 16)]
        if index in green_list:
            return True
        else:
            return False

class TestControllerIOYellow(object):
    def is_cob_on(self, index):
        yellow_list = [i for i in range(16, 32)]
        if index in yellow_list:
            return True
        else:
            return False

class TestControllerIORed(object):
    def is_cob_on(self, index):
        red_list = [i for i in range(32, 48)]
        if index in red_list:
            return True
        else:
            return False

class TestEvcConnector(unittest.TestCase):
    def setUp(self):
        self.asc3app_path = Path(os.path.abspath(os.path.join(__file__, "..", "asc3app")))
        self.evc_sumo_cfg_path = Path(os.path.abspath(os.path.join(__file__, "..", "resources", "test_evc_sumo_cfg.json")))

    def test_get_controller_cfg_path_list(self):
        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)
        evc_connector.get_controller_cfg_path_list()
        self.assertEqual(len(evc_connector.controller_cfg_path_list), 2)

    def test_cob_to_traffic_light_status(self):
        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)
        phase = 1
        ## get green light string
        green_io = TestControllerIOGreen()
        state = evc_connector.COB_to_traffic_light_status(green_io, phase)
        self.assertEqual(state, "g")
        ## get yellow light string
        yellow_io = TestControllerIOYellow()
        state = evc_connector.COB_to_traffic_light_status(yellow_io, phase)
        self.assertEqual(state, "y")
        ## get red light string
        red_io = TestControllerIORed()
        state = evc_connector.COB_to_traffic_light_status(red_io, phase)
        self.assertEqual(state, "r")

    def test_get_traffic_light_status(self):
        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)

        ## get green light state string
        green_io = TestControllerIOGreen()
        state = evc_connector.get_traffic_light_status_from_EVC(green_io, [{"index": [0, 1], "phaseId": 1}, {"index": [2, 3], "phaseId": 2}])
        self.assertEqual(state, "gggg")
        ## get yellow light state string
        yellow_io = TestControllerIOYellow()
        state = evc_connector.get_traffic_light_status_from_EVC(yellow_io, [{"index": [0, 1], "phaseId": 1}, {"index": [2, 3], "phaseId": 2}])
        self.assertEqual(state, "yyyy")
        ## get red light state string
        red_io = TestControllerIORed()
        state = evc_connector.get_traffic_light_status_from_EVC(red_io, [{"index": [0, 1], "phaseId": 1}, {"index": [2, 3], "phaseId": 2}])
        self.assertEqual(state, "rrrr")

if __name__ == '__main__':
    unittest.main()
