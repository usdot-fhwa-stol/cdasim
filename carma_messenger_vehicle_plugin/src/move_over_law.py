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

    def __init__(self, sumo_connector):

        config = configparser.ConfigParser()
        config.read('resources/move_over_law_cfg.ini')
        self._closure_uptrack = config.get('Settings', 'closure_uptrack')
        self._closure_downtrack = config.get('Settings', 'closure_downtrack')
        self._target_veh_id = config.get('Settings', 'target_id')
        self._veh_id = config.get('Settings', 'vehicle_id')
        self._min_gap = config.get('Settings', 'min_gap')
        self._advisory_speed_limit = config.get('Settings', 'advisory_speed_limit')
        self.sumo_connector = sumo_connector

    def close_lane(self):
        #send lane closure message
        combined_string = self._closure_uptrack + ";" + self._closure_downtrack + ";" + self._min_gap + ";" + self._advisory_speed_limit
        self.sumo_connector.set_parameter(self._veh_id, 'VehicleBroadcastTrafficEvent', combined_string)


    def park_messenger(self):
        target_lane = self.sumo_connector.get_target_lane(self._veh_id, True)
        self.sumo_connector.move_veh_lane(self._veh_id, target_lane)
        self.sumo_connector.stop_veh(self._veh_id)
        self.close_lane()
        return

    def get_closer(self):
        target_lane = self.sumo_connector.get_target_lane(self._veh_id, False)
        self.sumo_connector.move_veh_lane(self._veh_id, target_lane)
        return

    def move_over(self):
        veh_pos = self.sumo_connector.get_veh_pos(self._veh_id)
        target_pos = self.sumo_connector.get_veh_pos(self._target_veh_id)
        distance = self.sumo_connector.cal_distance(veh_pos, target_pos)

        if distance < 10:
            self.park_messenger()
            self.close_lane()
        elif distance < 600:
            self.get_closer()
