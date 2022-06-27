#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import shutil
import tempfile

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can not find carla library .egg")

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import sumolib  # pylint: disable=wrong-import-position
import traci  # pylint: disable=wrong-import-position

from carla_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from carla_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

from carla_integration.synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position

from util.netconvert_carla import netconvert_carla
# ==================================================================================================
# -- main ------------------------------------------------------------------------------------------
# ==================================================================================================


def main(args):

    # ----------------
    # carla simulation
    # ----------------
    carla_simulation = CarlaSimulation(args.host, args.port, args.map, args.step_length)
    
    # ---------------
    # sumo simulation
    # ---------------
    if args.net_file is not None:
        sumo_net=sumolib.net.readNet(args.net_file)
        print("load net file")
    else:
        # Temporal folder to save intermediate files.
        tmpdir = tempfile.mkdtemp()    
        current_map = carla_simulation.client.world.get_map()
        xodr_file = os.path.join(tmpdir, current_map.name + '.xodr')
        current_map.save_to_disk(xodr_file)
        net_file = os.path.join(tmpdir, current_map.name + '.net.xml')
        netconvert_carla(xodr_file, net_file, guess_tls=True)
        sumo_net = sumolib.net.readNet(net_file)

    sumo_simulation = SumoSimulation(sumo_net,
                                     args.step_length,
                                     host=args.bridge_server_host,
                                     port=args.bridge_server_port)

    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)

    # start simulation synchronization
    try:
        while True:
            synchronization.tick()

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')
    except traci.exceptions.FatalTraCIError:
        print("Socket server closed")

    finally:
        try: 
            synchronization.close()
        except:
            print("Connection closed")
        if "tmpdir" in locals():
            if os.path.exists(tmpdir):
                shutil.rmtree(tmpdir)



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the CARLA host server (default: 127.0.0.1)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='CARLA TCP port to listen to (default: 2000)')
    argparser.add_argument('--bridge-server-host',
                           default='localhost',
                           help='IP of the bridge host server (default: None)')
    argparser.add_argument('--bridge-server-port',
                           default=8913,
                           type=int,
                           help='TCP port to listen to (default: None)')
    argparser.add_argument('-m', 
                           '--map',
                            help='load a new map')
    argparser.add_argument('net_file', 
                           type=str,
                           default=None,
                           help='load the net file')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    args = argparser.parse_args()

    if args.sync_vehicle_all is True:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    main(args)
