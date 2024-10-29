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

    def __init__(self):

        config = configparser.ConfigParser()
        config.read('resources/move_over_law_cfg.ini')
        self._closure_uptrack = config.get('Settings', 'closure_uptrack')
        self._closure_downtrack = config.get('Settings', 'closure_downtrack')
        self._target_location = config.get('Settings', 'location')
        self._veh_id = config.get('Settings', 'vehicleID')
        self._event_reason = 'Move Over Law'
        self._event_type = 'CLOSED'

    def close_lane(self):
        #send lane closure message
        SumoConnector.set_parameter(self._veh_id,'event_reason', self._event_reason)
        SumoConnector.set_parameter(self._veh_id, 'event_type', self._event_type)
        SumoConnector.set_parameter(self._veh_id, 'uptrack', self._closure_uptrack)
        SumoConnector.set_parameter(self._veh_id, 'downtrack', self._closure_downtrack)

    def park_messenger(self):
        target_lane = SumoConnector.get_target_lane(self._veh_id, True)
        SumoConnector.move_veh_lane(self._veh_id, target_lane)
        SumoConnector.stop_veh(self._veh_id)
        self.close_lane()
        return

    def get_closer(self):
        target_lane = SumoConnector.get_target_lane(self._veh_id, False)
        SumoConnector.move_veh_lane(self._veh_id, target_lane)
        return
    
    def move_over(self):

        while True:

            distance = SumoConnector.cal_distance(self._veh_id, self._target_location)
            
            if distance < 10:
                self.park_messenger()
                break
            elif distance < 600:        
                self.get_closer()
    