import sys
import os
from pathlib import Path
import argparse

from evc_connector import EvcConnector
from sumo_connector import SumoConnector

def run(args):
    sumo_connector = SumoConnector(args.traci_port, args.traci_order_num)
    evc_connector = EvcConnector(args.asc3app_path, args.evc_sumo_cfg_path)
    evc_connector.run(sumo_connector)

if __name__ == "__main__":
    arg = argparse.ArgumentParser(description='EVC-SUMO integration')
    arg.add_argument('--asc3app-path',
                     required=True,
                     help='asc3app-application file path')
    arg.add_argument('--traci-port',
                     default=2000,
                     help='Traci port to connect to SUMO')
    arg.add_argument('--traci-order-num',
                     default=2,
                     help='Traci order number for SUMO multi-clients connection')
    arg.add_argument('--evc-sumo-cfg-path',
                     default='resources/controller_cfg.json')
    args = arg.parse_args()
    run(args)
