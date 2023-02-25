from pyeos.virtual.factory import virtual_factory
from pyeos.virtual import VirtualControllerOptions
import json

MAX_INTERSECTION_PHASES = 16

class EvcConnector:

    def tick(self):
        """
        Advance EVC controllers time via PyEOS
        """
        for harness in self.harness_list:
            harness.tick(1)

    def __init__(self, asc3app_path, evc_sumo_cfg_path):
        """
        Initialize
        -------------
        asc3app_path: The directory to asc3app which provided by Econolite team
        type: string
        -------------
        evc_sumo_cfg_path: The directory to evc_sumo_cfg.json file, default path is "resources/evc_sumo_cfg.json"
        type: string
        -------------
        """
        self.asc3app_path = asc3app_path
        self.evc_sumo_cfg_path = evc_sumo_cfg_path
        self.harness_list = []
        self.controller_io_list = []

    def detector_status_to_CIB(self):
        ## TBD
        pass

    def COB_to_traffic_light_status(self, controller_io, phase_id):
        """
        Convert COB status to string
        -------------
        controller_io: The controller input/output (cib/cob)
        type:
        -------------
        phase_id: The EVC traffic light phase ID, start from 1
        type: int
        -------------
        """

        ## is_cob_on(0) is to check if phase 1 is or not in green state
        ## is_cob_on(16) is to check if phase 1 is or not in yellow state
        ## is_cob_on(32) is to check if phase 1 is or not in red state
        if controller_io.is_cob_on( (phase_id - 1) + (MAX_INTERSECTION_PHASES * 2) ):
            return 'r'
        elif controller_io.is_cob_on( (phase_id - 1) + (MAX_INTERSECTION_PHASES * 1) ):
            return 'y'
        elif controller_io.is_cob_on( (phase_id - 1) + (MAX_INTERSECTION_PHASES * 0) ):
            return 'g'

    def get_traffic_light_status_from_EVC(self, controller_io, phases):
        """
        Get traffic light status from EVC to SUMO state string
        -------------
        controller_io: The controller input/output (cib/cob)
        type:
        -------------
        phases: Phases information defined in controller_cfg.json file
        type: dict
        -------------
        return: string
        """

        ## get number of characters in state string for SUMO
        state_num = 0
        for phase in phases:
            state_num = state_num + len(phase["index"])
        ## init sumo tl state string with all red states
        state_string = ['r'] * state_num

        for phase in phases:
            state_string = [self.COB_to_traffic_light_status(controller_io, phase['phaseId']) if i in phase['index'] else x for i,x in enumerate(state_string)]
        return ''.join(state_string)

    def set_detector_status_to_EVC(self):
        ## TBD
        pass

    def get_controller_cfg_path_list(self):
        """
        Store all controller config path into a list in order to pass to eos_factory to initialize controllers
        """
        with open(self.evc_sumo_cfg_path) as cfg_file:
            self.config_json = json.load(cfg_file)
            self.controller_cfg_path_list = []
            for controller_cfg_path in self.config_json['controllers']:
                self.controller_cfg_path_list.append(VirtualControllerOptions(controller_cfg_path['controllerCfgPath']))

    def run(self, sumo_connector):
        """
        main loop
        """
        ## init traci connection
        with sumo_connector.traci_handler() as traci_session:

            ## init EVC
            with virtual_factory(self.asc3app_path) as eos_factory:

                self.get_controller_cfg_path_list()

                ## init eos controller(s)
                with eos_factory.run_multiple(self.controller_cfg_path_list) as eos_controllers:

                    ## enable/disable EVC web panel
                    for i in range(len(eos_controllers)):
                        if self.config_json['controllers'][i]['enableWebPanel']:
                            eos_controllers[i].watch()
                            print("Launch EOS web pannel for controller ID:", self.config_json['controllers'][i]['controllerId'], ", SUMO TLID:", self.config_json['controllers'][i]['sumoTlId'])
                        else:
                            continue

                    ## init eos harness
                    with eos_factory.eos_harness(eos_controllers) as harnesses:

                        ## store io and harness into list
                        ## harness needs to be used to advance controller time
                        ## io needs to be used to get CIB/COB
                        for i in range(len(harnesses)):
                            io = harnesses[i].io()
                            harnesses[i].tick(1)
                            self.controller_io_list.append(io)
                            self.harness_list.append(harnesses[i])

                        while True:
                            sumo_connector.tick()
                            for i in range(len(self.controller_io_list)):
                                tl_state_string = self.get_traffic_light_status_from_EVC(self.controller_io_list[i], self.config_json['controllers'][i]['phases'])
                                sumo_connector.set_traffic_light_status_to_SUMO(self.config_json['controllers'][i]['sumoTlId'], tl_state_string)
                            self.tick()
