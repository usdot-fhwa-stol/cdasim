#!/usr/bin/env python
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('/home/carla/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import numpy as np
import json
import random
import pygame
import socket

array_image = np.empty([800,600,3])

def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)

def camera_callback(image):
    global array_image
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    array_image = array

def draw_image(surface, array):
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))

def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    pygame.init()
    display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)

    try:
        world = client.load_world('Town02')
        world.set_weather(carla.WeatherParameters(
            cloudiness=10.0,
            precipitation=10.0,
            sun_azimuth_angle=90,
            sun_altitude_angle=90))
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        blueprints = world.get_blueprint_library().filter(args.filterv)
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        # -----------------------------------------------------------------------------
        # -------------
        # EGO VEHICLE
        # -------------
        # some settings
        m = world.get_map()
        start_pose = carla.Transform(carla.Location(35, 302.5, 0.5),carla.Rotation(0,-180,0))
        ego_vehicle = world.try_spawn_actor(random.choice(blueprint_library.filter('vehicle.toyota.prius')), start_pose)
        batch.append(SpawnActor(blueprint, start_pose).then(SetAutopilot(ego_vehicle, False)))
        ego_vehicle.set_simulate_physics(True)

        bp_camera = blueprint_library.find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        bp_camera.set_attribute('image_size_x', '800')
        bp_camera.set_attribute('image_size_y', '600')
        bp_camera.set_attribute('fov', '110')
        transform = carla.Transform(carla.Location(x=-5.5, z=2.5))
        camera_rgb = world.spawn_actor(bp_camera, transform, attach_to=ego_vehicle)
        camera_rgb.listen(lambda image: camera_callback(image=image))

        print('Camera created')

        while True:
            if should_quit():
                return
            clock.tick()
            world.tick()
            world_snapshot = world.get_snapshot()           
            
            draw_image(display, array_image)
            display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
            pygame.display.flip()

    finally:
        # if args.sync and synchronous_master:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        carla.command.DestroyActor(camera_rgb.id)
        carla.command.DestroyActor(ego_vehicle.id)
        pygame.quit()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
