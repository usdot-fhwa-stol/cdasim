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

from pyeos.virtual.factory import virtual_factory
from pyeos.virtual import VirtualControllerOptions
import json

MAX_INTERSECTION_PHASES = 16

class EvcConnector:

    def tick(self, advance_steps):
        """
        Advance EVC controllers time via PyEOS
        """
        for harness in self.harness_list:
            harness.tick(advance_steps)

    def __init__(self, asc3app_path, evc_sumo_cfg_path):
        """
        Initialize EvcConnector object

        Parameters:
        - asc3app_path: The directory to asc3app which is provided by the Econolite team
        type: string

        - evc_sumo_cfg_path: The directory to evc_sumo_cfg.json file, default path is "resources/evc_sumo_cfg.json"
        type: string
        """
        self.asc3app_path = asc3app_path
        self.evc_sumo_cfg_path = evc_sumo_cfg_path
        self.harness_list = []
        self.controller_io_list = []

    def COB_to_traffic_light_status(self, controller_io, evc_phase_id):
        """
        Convert COB status to string

        Parameters:
        - controller_io: The controller input/output (cib/cob)
        type: pyeos.common.harness

        - evc_phase_id: The EVC traffic light phase ID, start from 1
        type: int
        """
        ## This function converts the COB status for a given traffic light phase to a string representing its state (red, yellow, or green).
        ## The controller input/output (cib/cob) is passed as an argument along with the phase ID.
        ## The function returns a string representing the state of the traffic light phase.

        ## is_cob_on(0) is to check if phase 1 is or not in green state
        ## is_cob_on(16) is to check if phase 1 is or not in yellow state
        ## is_cob_on(32) is to check if phase 1 is or not in red state
        ## MAX_INTERSECTION_PHASES default setting is value 16 as setting in line 20
        if controller_io.is_cob_on( (evc_phase_id - 1) + (MAX_INTERSECTION_PHASES * 2) ):
            return 'r'
        elif controller_io.is_cob_on( (evc_phase_id - 1) + (MAX_INTERSECTION_PHASES * 1) ):
            return 'y'
        elif controller_io.is_cob_on( (evc_phase_id - 1) + (MAX_INTERSECTION_PHASES * 0) ):
            return 'g'
        else:
            ## if state is not r, y or g then return O which is switched off state
            return 'O'

    def get_traffic_light_state_from_EVC(self, controller_io, phases):
        """
        Get traffic light status from EVC to SUMO state string

        Parameters:
        - controller_io: The controller input/output (cib/cob)
        type: pyeos.common.harness

        - phases: Phases information defined in evc_sumo_cfg.json file
        type: dict

        Returns:
        - string: A string representing the current state of the traffic light. 
                  The possible characters are "r" for red, "y" for yellow, "g" for green and "O" for switched off
        """

        ## get number of characters in state string for SUMO and init sumo tl state string with all red
        state_string = ['r'] * sum(len(phase['sumoTlStateIndex']) for phase in phases)

        for phase in phases:
            state_string = [self.COB_to_traffic_light_status(controller_io, phase['evcPhaseId']) if i in phase['sumoTlStateIndex'] else x for i,x in enumerate(state_string)]
        return ''.join(state_string)

    def set_induction_loop_status_to_EVC(self, controller_io, evc_phase_id, sumo_induction_loop_status):
        """
        Set induction loop status from SUMO to EVC via PyEOS CIB

        Parameters:
        - controller_io: The controller input/output (cib/cob)
        type: pyeos.common.harness

        - evc_phase_id: EVC phase ID
        type: int

        - sumo_induction_loop_status: induction loop status retrieved from SumoConnector
        type: int

        Returns:
        - int: return an integer to represent cib is on or off.
               cib on: 1
               cib off: 0
        """
        if sumo_induction_loop_status != 0:
            if controller_io.is_cib_off(evc_phase_id - 1):
                ## CIB is currently off, turn to on
                controller_io.cib_on(evc_phase_id - 1)
                return 1
            else:
                ## CIB is currently on
                return 1
        else:
            if controller_io.is_cib_on(evc_phase_id - 1):
                ## CIB is currently on, turn to off
                controller_io.cib_off(evc_phase_id - 1)
                return 0
            else:
                ## CIB is currently off
                return 0

    def get_controller_cfg_list(self):
        """
        Get the controller config path list
        """
        controller_cfg_list = []
        with open(self.evc_sumo_cfg_path) as cfg_file:
            self.config_json = json.load(cfg_file)
            for controller_cfg_path in self.config_json['controllers']:
                option = VirtualControllerOptions(
                            cfg_path=controller_cfg_path['controllerCfgPath'],
                            start_time=controller_cfg_path['start_time'],
                            https_port=controller_cfg_path['https_port'],
                            web_port=controller_cfg_path['web_port'],
                            snmp_port=controller_cfg_path['snmp_port'],
                            harness_port=controller_cfg_path['harness_port'],
                            controller_speed=controller_cfg_path['controller_speed'])
                controller_cfg_list.append(option)
        return controller_cfg_list


    def run(self, sumo_connector):
        """
        The main loop for the EVC connector
        -------------
        sumo_connector: The SUMO connector instance
        type: object
        -------------
        """
        ## init traci connection
        with sumo_connector.traci_handler() as traci_session:

            ## init EVC
            with virtual_factory(self.asc3app_path) as eos_factory:

                ## init eos controller(s)
                with eos_factory.run_multiple(self.get_controller_cfg_list()) as eos_controllers:

                    ## enable/disable EVC web panel
                    for i in range(len(eos_controllers)):
                        if self.config_json['controllers'][i]['enableWebPanel']:
                            eos_controllers[i].watch()
                            print("Launch EOS web panel for controller ID:", self.config_json['controllers'][i]['controllerId'], ", SUMO TLID:", self.config_json['controllers'][i]['sumoTlId'])
                        else:
                            continue

                    ## init eos harness
                    with eos_factory.eos_harness(eos_controllers) as harnesses:

                        ## store io and harness into list
                        ## harness needs to be used to advance controller time
                        ## io needs to be used to get CIB/COB
                        for i in range(len(harnesses)):
                            io = harnesses[i].io()

                            ## harness.tick is a fix step length, 0.1, with given value update times
                            ## if harness.tick(1) which means advance EVC 1 time within 0.1 second
                            ## if harness.tick(10) which means advance EVC 10 time within 0.1 second
                            ## By retrieving step length from SUMO (which is configured in MOSAIC configuration file)
                            ## the formula to convert the value for EVC will be 1 / (sumo_step_length * 10)
                            harnesses[i].tick( int(1 / (sumo_connector.traci_get_step_length() * 10)) )
                            self.controller_io_list.append(io)
                            self.harness_list.append(harnesses[i])

                        ground_truth_sumo_loop_detector_ids = sumo_connector.get_induction_loop_id_list()
                        while True:
                            sumo_connector.tick()
                            for i, controller in enumerate(self.config_json['controllers']):
                                ## Update EVC loop detector status
                                for evcPhase in controller['evcPhases']:
                                    ## Some intersection or lane might not set loop detector
                                    if 'sumoInductionLoopId' not in evcPhase:
                                        continue
                                    ## Check if induction loop id in config file exists in SUMO simulation
                                    if evcPhase['sumoInductionLoopId'] in ground_truth_sumo_loop_detector_ids:
                                        sumo_induction_loop_status = sumo_connector.get_induction_loop_status_from_SUMO(evcPhase['sumoInductionLoopId'])
                                        self.set_induction_loop_status_to_EVC(self.controller_io_list[i], evcPhase['evcPhaseId'], sumo_induction_loop_status)

                                ## Update SUMO traffic light state
                                tl_state_string = self.get_traffic_light_state_from_EVC(self.controller_io_list[i], controller['evcPhases'])
                                sumo_connector.set_traffic_light_state_to_SUMO(controller['sumoTlId'], tl_state_string)

                                
                            self.tick( int(1 / (sumo_connector.traci_get_step_length() * 10)) )
