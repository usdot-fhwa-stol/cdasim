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

import sys
import os
import logging
import math

if 'SUMO_HOME' in os.environ:
    tools_path = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools_path)
    import traci
else:
    sys.exit(1)

from contextlib import contextmanager

class SumoConnector:

    def _setup_logging(self, level):
        """
        Configures the logging level for the SumoConnector.

        Parameters:
        - level: Desired logging level as a string (e.g., 'DEBUG', 'INFO').
        """
        numeric_level = getattr(logging, level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % level)
        logging.basicConfig(level=numeric_level, format='%(asctime)s - %(levelname)s - %(message)s')

    def __init__(self, ip, port, order_num, log_level='INFO'):
        """
        Initializes the SumoConnector object which manages the connection to SUMO via TraCI.

        Parameters:
        - ip: IP address for Traci to connect to SUMO.
        type: str
        - port: The port number to connect to SUMO, which should be consistent with SUMO simulator setting.
        type: int
        - order_num: The priority order number of the connection to SUMO, useful for multi-client setups.
        type: int
        - log_level: The logging level for the connector's operations.
        type: str
        """
        self._ip = ip
        self._port = port
        self._order_num = order_num
        self._traci = traci
        self._setup_logging(log_level)

    @contextmanager
    def traci_handler(self):
        """
        Context manager that initializes and closes the TraCI connection.
        """
        try:
            self._traci.init(host=self._ip, port=self._port)
            self._traci.setOrder(self._order_num)
            yield
        finally:
            self._traci.close()

    def tick(self):
        """
        Advances the SUMO simulation by one time step.
        """
        try:
            self._traci.simulationStep()
        except Exception as e:
            logging.error(f"Error during simulation step: {e}")
            raise

    def add_veh(self, veh_id, route_id="", depart=0):
        """
        Adds a vehicle to the simulation with specified parameters.

        Parameters:
        - veh_id: Identifier for the vehicle to add.
        - route_id: Route identifier the vehicle should follow.
        - depart: Departure time for the vehicle.
        """
        try:
            self._traci.vehicle.add(vehID=veh_id, routeID=route_id, depart=depart)
        except Exception as e:
            logging.error(f"Failed to add vehicle {veh_id}: {e}")
            raise

    def get_veh_ids(self):
        """
        Retrieves a list of all current vehicle IDs in the simulation.
        """
        try:
            return self._traci.vehicle.getIDList()
        except Exception as e:
            logging.error(f"Failed to retrieve vehicle IDs: {e}")
            raise

    def set_veh_speed(self, veh_id, target_speed):
        """
        Sets the speed of a specified vehicle.

        Parameters:
        - veh_id: The ID of the vehicle.
        - target_speed: New target speed for the vehicle.
        """
        try:
            self._traci.vehicle.setSpeed(veh_id, target_speed)
        except Exception as e:
            logging.error(f"Failed to set speed for vehicle {veh_id}: {e}")
            raise

    def set_veh_route(self, veh_id, full_route):
        """
        Sets the route for a specified vehicle.

        Parameters:
        - veh_id: The ID of the vehicle.
        - full_route: A list of edge IDs defining the route.
        """
        try:
            self._traci.vehicle.setRoute(veh_id, full_route)
        except Exception as e:
            logging.error(f"Failed to set route for vehicle {veh_id}: {e}")
            raise

    def set_veh_lcm(self, veh_id, mode):
        """
        Sets the lane change mode for a specified vehicle.

        Parameters:
        - veh_id: The ID of the vehicle.
        - mode: Lane change mode bitmask to apply.
        """
        try:
            self._traci.vehicle.setLaneChangeMode(veh_id, mode)
        except Exception as e:
            logging.error(f"Failed to set lane change mode for vehicle {veh_id}: {e}")
            raise

    def get_sim_time(self):
        """
        Retrieves the current simulation time from SUMO.
        """
        try:
            return self._traci.simulation.getTime()
        except Exception as e:
            logging.error("Failed to get simulation time: " + str(e))
            raise

    def set_veh_type(self, veh_id, model):
        """
        Sets the vehicle type for a specified vehicle based on the given model.
        The vehicle type determines the car-following model to be used by the vehicle in the simulation.
        The vehicle type must be predefined in the SUMO .rou.xml configuration file of the CDASim scenario.

        Parameters:
        - veh_id: The identifier for the vehicle whose type is to be set.
        - model: The vehicle type identifier, which must correspond to one of the vehicle types defined in the .rou.xml file.

        Raises:
        - Exception: If there is an error in setting the vehicle type, such as if the vehicle ID does not exist or the model is not defined.
        """
        try:
            self._traci.vehicle.setType(veh_id, model)
        except Exception as e:
            logging.error(f"Failed to set vehicle model for vehicle ID '{veh_id}': {e}")
            raise

    def set_veh_signal(self, veh_id, signal):
        """
        Sets the vehicle signals for a specified vehicle in the SUMO simulation.
        This function is used to control visual indicators like emergency lights on a vehicle.
        The signal parameter is an integer that encodes the state of the vehicle’s signals according to the SUMO vehicle signaling documentation.

        The function is particularly useful for integrations where external systems (such as MOSAIC) monitor these signals for various purposes,
        including activating functionalities like the Broadcast of Basic Safety Messages (BSM) in connected vehicle environments.

        See the SUMO documentation for detailed signal encoding: https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html

        Parameters:
        - veh_id: str, the identifier for the vehicle whose signals are to be set.
        - signal: int, an integer encoding the state of the vehicle's signals.

        Raises:
        - Exception: If there is an error in setting the vehicle signals, such as if the vehicle ID does not exist or the signal parameter is incorrect.

        """
        try:
            self._traci.vehicle.setSignals(veh_id, signal)
        except Exception as e:
            logging.error(f"Failed to set vehicle signal for vehicle ID '{veh_id}': {e}")
            raise

    def cal_distance(self, pos_1, pos_2):
        """
        Calculates the distance between a certain location and a vehicle in SUMO
        """
        try:
            distance = math.sqrt((pos_1[0] - pos_2[0])**2 + (pos_1[1] - pos_2[1])**2)
            return distance
        except Exception as e:
            logging.error(f"Failed to calculate vehicle distance for vehicle ID '{veh_id}': {e}")
            raise

    def stop_veh(self, veh_id):
        """
        stops vehicle at current place in SUMO
        """
        try:
            current_veh_pos = traci.vehicle.getLanePosition(veh_id)
            current_veh_edge = traci.vehicle.getRoadID(veh_id)
            current_veh_lane = traci.vehicle.getLaneIndex(veh_id)

            traci.vehicle.setStop(
            vehID=veh_id,
            edgeID=current_veh_edge,
            pos=current_veh_pos,
            laneIndex=current_veh_lane,
            duration=5,
            flags=traci.constants.STOP_FLAG_PARKING
            )
        except Exception as e:
            logging.error(f"Failed to stop vehicle for vehicle ID '{veh_id}': {e}")
            raise

    def move_veh_lane(self, veh_id, target_lane):

        try:
            traci.vehicle.setLaneChangeMode(veh_id, 0b001001011)

            while traci.vehicle.getLaneIndex(veh_id) != target_lane:
                current_lane = traci.vehicle.getLaneIndex(veh_id)

                if current_lane < target_lane:
                    traci.vehicle.changeLane(veh_id, current_lane + 1, 5)
                elif current_lane > target_lane:
                    traci.vehicle.changeLane(veh_id, current_lane - 1, 5)

        except Exception as e:
            logging.error(f"Failed to change vehicle lane for vehicle ID '{veh_id}': {e}")
            raise

    def get_target_lane(self, veh_id, is_rightmost):

        current_veh_edge = traci.vehicle.getRoadID(veh_id)
        num_lanes = traci.edge.getLaneNumber(current_veh_edge)

        if is_rightmost:
            return num_lanes - 1
        else:
            return num_lanes - 2

    def set_parameter(self, veh_id, para_name, para_value):

        try:
            traci.vehicle.setParameter(veh_id, para_name, para_value)

        except Exception as e:
            logging.error(f"Failed to set vehicle parameter for vehicle ID '{veh_id}': {e}")
            raise

    def get_veh_pos(self, veh_id):

        try:
            pos = traci.vehicle.getPosition(veh_id)
            return pos
        except Exception as e:
            logging.error(f"Failed to get vehicle position for vehicle ID '{veh_id}': {e}")
            raise
    
    def create_stop_veh(self, veh_id, end_pos, stop_route):
        try:
            traci.vehicle.add(vehID = veh_id, routeID=stop_route, typeID="car", depart=0, departPos=end_pos)
            traci.vehicle.setSpeed(veh_id, 0)
        except Exception as e:
            logging.error(f"Failed to create stopped vehicle for vehicle ID '{veh_id}': {e}")
            raise