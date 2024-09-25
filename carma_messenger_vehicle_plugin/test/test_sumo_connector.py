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
from unittest.mock import Mock, patch

import sys
sys.path.append('../src')
from sumo_connector import SumoConnector

class TestSumoConnector(unittest.TestCase):
    def setUp(self):
        # Patch the traci module to prevent actual SUMO calls
        self.traci_mock = patch('sumo_connector.traci').start()
        # Setup a SumoConnector instance with mocked traci
        self.connector = SumoConnector('localhost', 12345, 1, 'INFO')

    def tearDown(self):
        # Stop all patches
        patch.stopall()

    def test_init(self):
        # Test initialization and logging setup
        self.connector._setup_logging('INFO')
        self.traci_mock.init.assert_not_called()  # Ensures traci.init is not called upon initialization

    def test_traci_handler(self):
        # Test the traci context manager
        with self.connector.traci_handler():
            self.traci_mock.init.assert_called_once()
        self.traci_mock.close.assert_called_once()

    def test_tick(self):
        # Test the tick method advances the simulation
        self.connector.tick()
        self.traci_mock.simulationStep.assert_called_once()

    def test_add_veh(self):
        # Test adding a vehicle
        self.connector.add_veh('vehicle_1', 'route_1', 0)
        self.traci_mock.vehicle.add.assert_called_once_with(vehID='vehicle_1', routeID='route_1', depart=0)

    def test_get_veh_ids(self):
        # Test retrieving vehicle IDs
        self.connector.get_veh_ids()
        self.traci_mock.vehicle.getIDList.assert_called_once()

    def test_set_veh_speed(self):
        # Test setting vehicle speed
        self.connector.set_veh_speed('vehicle_1', 10.5)
        self.traci_mock.vehicle.setSpeed.assert_called_once_with('vehicle_1', 10.5)

    def test_set_veh_route(self):
        # Test setting vehicle route
        self.connector.set_veh_route('vehicle_1', ['road_1', 'road_2'])
        self.traci_mock.vehicle.setRoute.assert_called_once_with('vehicle_1', ['road_1', 'road_2'])

    def test_set_veh_lcm(self):
        # Test setting lane change mode
        self.connector.set_veh_lcm('vehicle_1', 0b0101)
        self.traci_mock.vehicle.setLaneChangeMode.assert_called_once_with('vehicle_1', 0b0101)

    def test_get_sim_time(self):
        # Test retrieving simulation time
        self.connector.get_sim_time()
        self.traci_mock.simulation.getTime.assert_called_once()
    
    def test_set_veh_type(self):
        # Test setting vehicle type
        self.connector.set_veh_type('vehicle_1', 'FreightERV')
        self.traci_mock.vehicle.setType('vehicle_1', 'FreightERV')

if __name__ == '__main__':
    unittest.main()
