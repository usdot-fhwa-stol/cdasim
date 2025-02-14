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

import unittest
from unittest.mock import patch, MagicMock, mock_open
import json

import sys
sys.path.append('../src')
from msger_veh_cfg import MsgerVehicleCfg

class TestMsgerVehicleCfg(unittest.TestCase):
    def setUp(self):
        # Setup mock JSON content, this content mirrors what you expect to be in the actual JSON file
        # It defines two vehicle configurations for testing purposes.
        self.mock_json_content = json.dumps({
            "msgerVehs": [
                {
                    "id": "vehicle_1",
                    "route": [
                        "1_26704526_26704482",
                        "32909782_26704482_26785753",
                        "25185001_26785753_26704584",
                        "25185007_26704584_21487146",
                        "25185006_21487146_21487168",
                        "4068038_21487168_251150126",
                        "4068038_251150126_428788319"
                    ],
                    "speed": 15,
                    "departureTime": 5,
                    "lcm": 0,
                    "cfm": "FreightERV"
                }
            ]
        })

    @patch('builtins.open', new_callable=mock_open)
    @patch('json.load')
    def test_vehicle_configuration(self, mock_load, mock_file):
        """
        Tests if the MsgerVehicleCfg class correctly parses the JSON configuration
        for vehicles and creates an internal representation. This test checks if the
        number of vehicle IDs matches the JSON input and validates specific properties
        of the vehicles like speed, route, and departure time.
        """
        # Set the mock return value for json.load to simulate loading of JSON from a file
        mock_load.return_value = json.loads(self.mock_json_content)
        # Initialize the MsgerVehicleCfg class with a mocked file path
        cfg = MsgerVehicleCfg("resources/msger_veh_cfg.json")
        # Assertions to ensure the vehicle data is parsed and stored correctly
        self.assertEqual(len(cfg.get_veh_ids()), 1)
        self.assertIn("vehicle_1", cfg.get_veh_ids())
        self.assertEqual(cfg.get_veh_speed("vehicle_1"), 15)
        self.assertEqual(cfg.get_veh_departure_time("vehicle_1"), 5)
        self.assertEqual(cfg.get_veh_route("vehicle_1"), [
            "1_26704526_26704482",
            "32909782_26704482_26785753",
            "25185001_26785753_26704584",
            "25185007_26704584_21487146",
            "25185006_21487146_21487168",
            "4068038_21487168_251150126",
            "4068038_251150126_428788319"])
        self.assertEqual(cfg.get_veh_cfm("vehicle_1"), "FreightERV")

if __name__ == '__main__':
    unittest.main()
