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

import argparse
import logging
from sumo_connector import SumoConnector
from msger_veh_cfg import MsgerVehicleCfg
from msger_veh_cfg import VehicleState 

def setup_logging(level):
    numeric_level = getattr(logging, level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % level)
    logging.basicConfig(level=numeric_level, format='%(asctime)s - %(levelname)s - %(message)s')

def run(args):
    """
    Executes the vehicle simulation management. This function initializes the vehicle configurations,
    connects to SUMO via TraCI, and manages the simulation steps and vehicle states.

    Parameters:
    - args: Command line arguments containing settings for Traci connection and configuration path.
    type: Namespace
    """
    setup_logging(args.log_level)
    try:
        msger_veh_cfg = MsgerVehicleCfg(args.msger_veh_cfg_path, log_level=args.log_level)
        sumo_connector = SumoConnector(args.traci_ip, args.traci_port, args.traci_order_num, log_level=args.log_level)
        msg_veh_ids = msger_veh_cfg.get_veh_ids()
        with sumo_connector.traci_handler():
            while True:
                sumo_connector.tick()
                sumo_veh_ids = sumo_connector.get_veh_ids()
                for msg_veh_id in msg_veh_ids:

                    if msg_veh_id not in sumo_veh_ids and \
                       msger_veh_cfg.get_veh_state(msg_veh_id) == VehicleState.NOT_CREATED and \
                       msger_veh_cfg.get_veh_departure_time(msg_veh_id) <= sumo_connector.get_sim_time():
                        ### Init
                        logging.info("Adding new vehicle with ID: " + msg_veh_id)
                        sumo_connector.add_veh(msg_veh_id)
                        sumo_connector.set_veh_route(msg_veh_id, msger_veh_cfg.get_veh_route(msg_veh_id))
                        sumo_connector.set_veh_speed(msg_veh_id, msger_veh_cfg.get_veh_speed(msg_veh_id))
                        sumo_connector.set_veh_lcm(msg_veh_id, msger_veh_cfg.get_veh_lcm(msg_veh_id))
                        sumo_connector.set_veh_type(msg_veh_id, msger_veh_cfg.get_veh_cfm(msg_veh_id))
                        msger_veh_cfg.set_veh_state(msg_veh_id, VehicleState.CREATED_AND_DRIVING)
                    elif msg_veh_id not in sumo_veh_ids and msger_veh_cfg.get_veh_state(msg_veh_id) == VehicleState.CREATED_AND_DRIVING:
                        ### Remove
                        msger_veh_cfg.set_veh_state(msg_veh_id, VehicleState.FINISHED_AND_DESTROYED)
                        logging.info("Vehicle " + msg_veh_id + " finished route.")
                    
                    ## TODO
                    
    except Exception as e:
        logging.error(f"An error occurred during simulation: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to manage messenger vehicles in a SUMO simulation.')
    parser.add_argument('--traci-ip', default="localhost", help='IP address for Traci to connect to SUMO.')
    parser.add_argument('--traci-port', default=2010, help='Port number for Traci to connect to SUMO.')
    parser.add_argument('--traci-order-num', default=2, help='Traci connection order number for SUMO multi-client setup.')
    parser.add_argument('--msger-veh-cfg-path', default="resources/msger_veh_cfg.json", help='Path to the messenger vehicle configuration JSON file.')
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                        help='Set the logging level')
    args = parser.parse_args()
    run(args)
