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


import sys
import os
from pathlib import Path
import argparse

from evc_connector import EvcConnector
from sumo_connector import SumoConnector

def run(args):
    sumo_connector = SumoConnector(arg.traci_ip, args.traci_port, args.traci_order_num)
    evc_connector = EvcConnector(args.asc3app_path, args.evc_sumo_cfg_path)
    evc_connector.run(sumo_connector)

if __name__ == "__main__":
    arg = argparse.ArgumentParser(description='EVC-SUMO integration')
    arg.add_argument('--asc3app-path',
                     required=True,
                     help='asc3app-application file path')
    arg.add_argument('--traci-ip',
                     default="localhost",
                     help='Traci ip to connect to SUMO')
    arg.add_argument('--traci-port',
                     default=2010,
                     help='Traci port to connect to SUMO')
    arg.add_argument('--traci-order-num',
                     default=2,
                     help='Traci order number for SUMO multi-clients connection')
    arg.add_argument('--evc-sumo-cfg-path',
                     default='resources/evc_sumo_cfg.json')
    args = arg.parse_args()
    run(args)
