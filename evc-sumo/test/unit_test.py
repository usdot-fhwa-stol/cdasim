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
from unittest.mock import patch

sys.path.append('../src')
from evc_connector import EvcConnector

class TestControllerIOOff(object):
    def is_cib_on(self, index):
        return False
    def cib_on(self, index):
        return
    def is_cib_off(self, index):
        return True
    def cib_off(self, index):
        return

class TestControllerIOOn(object):
    def is_cib_on(self, index):
        return True
    def cib_on(self, index):
        return
    def is_cib_off(self, index):
        return False
    def cib_off(self, index):
        return

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

    def test_get_controller_cfg_list(self):
        """
        Test the functionality of the `get_controller_cfg_list` method of the `EvcConnector` class.
        
        This method tests if the `get_controller_cfg_list` method can correctly read and return the 
        controller configuration list from the specified path in the file system.
        """
        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)
        self.assertEqual(len(evc_connector.get_controller_cfg_list()), 2)

    def test_cob_to_traffic_light_status(self):
        """
        Test the functionality of the `COB_to_traffic_light_status` method of the `EvcConnector` class.
        
        This method tests if the `COB_to_traffic_light_status` method can correctly convert the 
        state of a Controller Output Bit (COB) to a traffic light state string ('g', 'y', 'r').
        The test cases use the `TestControllerIOGreen`, `TestControllerIOYellow`, and `TestControllerIORed`
        classes to simulate the COB states.
        """
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

    def test_get_traffic_light_state(self):
        """
        Test the functionality of the `get_traffic_light_state_from_EVC` method of the `EvcConnector` class.
        
        This method tests if the `get_traffic_light_state_from_EVC` method can correctly read and return the 
        state of the traffic lights for the specified phases from the EVC.
        The test cases use the `TestControllerIOGreen`, `TestControllerIOYellow`, and `TestControllerIORed`
        classes to simulate the COB states.
        """
        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)

        ## get green light state string
        green_io = TestControllerIOGreen()
        state = evc_connector.get_traffic_light_state_from_EVC(green_io, [{"sumoTlStateIndex": [0, 1], "evcPhaseId": 1}, {"sumoTlStateIndex": [2, 3], "evcPhaseId": 2}])
        self.assertEqual(state, "gggg")
        ## get yellow light state string
        yellow_io = TestControllerIOYellow()
        state = evc_connector.get_traffic_light_state_from_EVC(yellow_io, [{"sumoTlStateIndex": [0, 1], "evcPhaseId": 1}, {"sumoTlStateIndex": [2, 3], "evcPhaseId": 2}])
        self.assertEqual(state, "yyyy")
        ## get red light state string
        red_io = TestControllerIORed()
        state = evc_connector.get_traffic_light_state_from_EVC(red_io, [{"sumoTlStateIndex": [0, 1], "evcPhaseId": 1}, {"sumoTlStateIndex": [2, 3], "evcPhaseId": 2}])
        self.assertEqual(state, "rrrr")
            
    def test_set_induction_loop_status_to_EVC(self):
        """
        Test the functionality of the `set_induction_loop_status_to_EVC` method of the `EvcConnector` class.
        
        This method tests if the `set_induction_loop_status_to_EVC` method can correctly set the induction loop status 
        to the EVC for both on and off states. The test cases use the `TestControllerIOOn` and `TestControllerIOOff`
        classes to simulate the controller inputs for the induction loop state.
        """

        evc_connector = EvcConnector(self.asc3app_path, self.evc_sumo_cfg_path)
        
        # Create an instance of the TestControllerIOOn and TestControllerIOOff class to 
        # simulate the controller input for the induction loop state being on and off
        cib_on_io = TestControllerIOOn()
        cib_off_io = TestControllerIOOff()

        # Check if the `set_induction_loop_status_to_EVC` method can correctly set the 
        # induction loop state to on for a given detector and lane index
        evc_phase_id = 1
        sumo_induction_loop_status = 1
        ## on
        self.assertEqual(evc_connector.set_induction_loop_status_to_EVC(cib_on_io, evc_phase_id, sumo_induction_loop_status), 1)
        self.assertEqual(evc_connector.set_induction_loop_status_to_EVC(cib_on_io, evc_phase_id, sumo_induction_loop_status), 1)
        sumo_induction_loop_status = 0
        ## off
        self.assertEqual(evc_connector.set_induction_loop_status_to_EVC(cib_off_io, evc_phase_id, sumo_induction_loop_status), 0)
        self.assertEqual(evc_connector.set_induction_loop_status_to_EVC(cib_off_io, evc_phase_id, sumo_induction_loop_status), 0)
        





if __name__ == '__main__':
    unittest.main()
