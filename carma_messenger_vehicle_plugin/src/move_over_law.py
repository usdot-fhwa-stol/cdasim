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

from sumo_connector import SumoConnector

import configparser

class MoveOverLaw:

    def __init__(self, sumo_connector, veh_id):

        config = configparser.ConfigParser()
        config.read('resources/move_over_law_cfg.ini')
        self._closure_uptrack = config.get('Settings', 'closure_uptrack')
        self._closure_downtrack = config.get('Settings', 'closure_downtrack')
        self._target_veh_id = config.get('Settings', 'target_id')
        self._veh_id = veh_id
        self._min_gap = config.get('Settings', 'min_gap')
        self._advisory_speed_limit = config.get('Settings', 'advisory_speed_limit')
        self._stop_route = config.get('Settings', 'stop_route')
        self._stop_pos = config.get('Settings', 'stop_pos')
        self._start_lane = config.get('Settings', 'start_lane')
        self.sumo_connector = sumo_connector
        self.sumo_connector.create_stop_veh(self._target_veh_id, self._stop_pos, self._stop_route)
        self.first_time_two_vehicles = True

    def close_lane(self):
        #send lane closure message
        combined_string = self._closure_uptrack + ";" + self._closure_downtrack + ";" + self._min_gap + ";" + self._advisory_speed_limit
        self.sumo_connector.set_parameter(self._veh_id, 'VehicleBroadcastTrafficEvent', combined_string)


    def park_messenger(self):
        target_lane = self.sumo_connector.get_veh_lane(self._target_veh_id)
        target_lane_index = int(target_lane.split('_')[-1])
        print(target_lane_index)
        self.sumo_connector.move_veh_lane(self._veh_id, target_lane_index)
        self.sumo_connector.stop_veh(self._veh_id)
        self.close_lane()
        return

    def get_closer(self):
        target_lane = self.sumo_connector.get_veh_lane(self._target_veh_id)
        lane_index = int(target_lane.split('_')[-1])
        target_lane_index = lane_index+1
        self.sumo_connector.move_veh_lane(self._veh_id, target_lane_index)
        return

    def move_over(self):
        if len(self.sumo_connector.get_veh_ids()) >=2:

            if self.first_time_two_vehicles:
                print(self.sumo_connector.get_veh_lane(self._veh_id))
                self.sumo_connector.set_veh_lane(self._veh_id, 1)
                self.first_time_two_vehicles = False

            veh_pos = self.sumo_connector.get_veh_pos(self._veh_id)
            target_pos = self.sumo_connector.get_veh_pos(self._target_veh_id)
            distance = self.sumo_connector.cal_distance(veh_pos, target_pos)
            print("Distance: " + str(distance))

            if distance < 30:
                self.park_messenger()
                self.close_lane()
            elif distance < 600:
                self.get_closer()
