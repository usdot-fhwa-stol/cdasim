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

import json
import logging
from enum import Enum

class VehicleState(Enum):
    NOT_CREATED = 0
    CREATED_AND_DRIVING = 1
    FINISHED_AND_DESTROYED = 2


class MsgerVehicleCfg:

    def _setup_logging(self, level):
        """
        Set up logging with the specified level.

        Parameters:
        - level: The log level to set up.
        type: str
        """
        numeric_level = getattr(logging, level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % level)
        logging.basicConfig(level=numeric_level, format='%(asctime)s - %(levelname)s - %(message)s')

    def __init__(self, cfg_path, log_level='INFO'):
        """
        Initializes the vehicle configuration manager by loading vehicle data from a JSON file.

        Each vehicle's configuration is stored in the dictionary `_msger_veh_dict`, keyed by the vehicle's ID. This dictionary includes:
        - 'route': list of strings representing the sequence of edges the vehicle will travel over in the simulation.
        - 'speed': float, the maximum speed of the vehicle in meters per second.
        - 'departureTime': int, the simulation time at which the vehicle starts.
        - 'lcm': int, an integer representing the lane change mode using SUMO's encoding.
        - 'cfm': str, the car following model that determines how the vehicle follows other vehicles.

        Parameters:
        - cfg_path: The file path to the JSON configuration file containing messenger vehicle data.
        type: str
        """

        self._setup_logging(log_level)
        self._msger_veh_dict = {}
        self._veh_state_dict = {}
        try:
            with open(cfg_path, 'r') as cfg_file:
                config_json = json.load(cfg_file)
                for veh in config_json["msgerVehs"]:
                    veh_id = veh["id"]
                    if veh_id in self._msger_veh_dict:
                        logging.warning(f"Duplicate vehicle ID detected and skipped: {veh_id}")
                        continue
                    self._msger_veh_dict[veh_id] = {
                        "route": veh["route"],
                        "speed": veh["speed"],
                        "departureTime": veh["departureTime"],
                        "lcm": veh["lcm"],
                        "cfm": veh["cfm"]
                    }
                    self._veh_state_dict[veh_id] = VehicleState.NOT_CREATED
            logging.info("Vehicle configuration loaded successfully.")
        except FileNotFoundError:
            logging.error("Configuration file not found. Please check the file path.")
        except json.JSONDecodeError:
            logging.error("Error decoding JSON. Please check the input JSON file for errors.")
        except Exception as e:
            logging.error(f"An unexpected error occurred: {e}")

    def get_veh_route(self, veh_id):
        """
        Retrieves the route for a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - list of str: The route associated with the vehicle.
        """
        return self._msger_veh_dict[veh_id]["route"]

    def get_veh_speed(self, veh_id):
        """
        Retrieves the speed setting for a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - float: The speed of the vehicle.
        """
        return self._msger_veh_dict[veh_id]["speed"]

    def get_veh_ids(self):
        """
        Retrieves a list of all vehicle IDs managed by this configuration.

        Returns:
        - list of str: A list of vehicle IDs.
        """
        return list(self._msger_veh_dict.keys())

    def get_veh_departure_time(self, veh_id):
        """
        Retrieves the departure time for a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - int: The departure time of the vehicle.
        """
        return self._msger_veh_dict[veh_id]["departureTime"]

    def get_veh_lcm(self, veh_id):
        """
        Retrieves the lane change mode setting for a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - int: The lane change mode of the vehicle.
        """
        return self._msger_veh_dict[veh_id]["lcm"]

    def get_veh_cfm(self, veh_id):
        """
        Retrieves the car-following model setting for a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - str: The car-following model of the vehicle.
        """
        return self._msger_veh_dict[veh_id]["cfm"]

    def set_veh_state(self, veh_id, state):
        """
        Sets the state of a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        - state: The new state to set for the vehicle.
        type: int
        """
        if veh_id in self._veh_state_dict:
            self._veh_state_dict[veh_id] = state
            logging.info("Set vehicle " + str(veh_id) + " state to " + state.name)
        else:
            logging.warning(f"Attempted to set state for a non-existent vehicle ID: {veh_id}")

    def get_veh_state(self, veh_id):
        """
        Retrieves the state of a specified vehicle ID.

        Parameters:
        - veh_id: The ID of the vehicle.
        type: str

        Returns:
        - int: The current state of the vehicle.
        """
        return self._veh_state_dict.get(veh_id, None)  # Returns None if the vehicle ID doesn't exist
