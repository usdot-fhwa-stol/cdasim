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

def parseSnapshot_2_dictionary(id, snapshot, isEgo):
    veh_data={}
    veh_data["id"] = id
    veh_data["global_x"] = snapshot.get_transform().location.x
    veh_data["global_y"] = snapshot.get_transform().location.y
    veh_data["global_z"] = snapshot.get_transform().location.z
    veh_data["rotation_roll"] = snapshot.get_transform().rotation.roll
    veh_data["rotation_pitch"] = snapshot.get_transform().rotation.pitch
    veh_data["rotation_yaw"] = snapshot.get_transform().rotation.yaw
    veh_data["velocity_x"] = snapshot.get_velocity().x
    if isEgo:
        veh_data["velocity_y"] = snapshot.get_velocity().y
        veh_data["velocity_z"] = snapshot.get_velocity().z
        veh_data["angular_x"] = snapshot.get_angular_velocity().x
        veh_data["angular_y"] = snapshot.get_angular_velocity().y
        veh_data["angular_z"] = snapshot.get_angular_velocity().z
        veh_data["accel_x"] = snapshot.get_acceleration().x
        veh_data["accel_y"] = snapshot.get_acceleration().y
        veh_data["accel_z"] = snapshot.get_acceleration().z
    return veh_data

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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
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
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
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
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enable')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []

    pygame.init()
    display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)

    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.settimeout(1000)
    #serv.bind(('128.32.234.153', 8080)) # 128.32.234.154
    serv.bind(('127.0.0.1', 8080))
    serv.listen(5)
    print("server passed")

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

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in client.apply_batch_sync(batch, True):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)
        # -----------------------------------------------------------------------------
        # -------------
        # EGO VEHICLE
        # -------------
        # some settings
        m = world.get_map()
        # start_pose = carla.Transform(carla.Location(35, 302.5, 0.5),carla.Rotation(0,-180,0))
        start_pose = carla.Transform(carla.Location(-3, 285, 0.5),carla.Rotation(0.14, -89.7,-0.04))

        ego_vehicle = world.try_spawn_actor(random.choice(blueprint_library.filter('vehicle.tesla.model3')), start_pose)
        batch.append(SpawnActor(blueprint, start_pose).then(SetAutopilot(ego_vehicle, False)))
        ego_vehicle.set_simulate_physics(True)

        bp_camera = blueprint_library.find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        bp_camera.set_attribute('image_size_x', '800')
        bp_camera.set_attribute('image_size_y', '600')
        bp_camera.set_attribute('fov', '110')
        transform = carla.Transform(carla.Location(x=-5.5, z=2.5))
        camera_rgb = world.spawn_actor(bp_camera, transform, attach_to=ego_vehicle)
        #camera_rgb.listen(lambda image: draw_image(display, image=image))
        camera_rgb.listen(lambda image: camera_callback(image=image))

        print('Camera created')

        # -----------------------------------------------------------------------------
        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers.' % (len(vehicles_list), len(walkers_list)))

        # example of how to use parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        ego_ID = ego_vehicle.id
        init_dict = {}
        init_dict["ego_info"] = {}
        init_dict["ego_state"] = {}

        init_dict["ego_id"] = ego_ID
        init_dict["map_id"] = 2

        init_dict["ego_info"]["x_length"] = ego_vehicle.bounding_box.extent.x
        init_dict["ego_info"]["y_length"] = ego_vehicle.bounding_box.extent.y
        init_dict["ego_info"]["z_length"] = ego_vehicle.bounding_box.extent.z

        init_dict["ego_state"]["x_pose"] = start_pose.location.x
        init_dict["ego_state"]["y_pose"] = start_pose.location.y
        init_dict["ego_state"]["z_pose"] = start_pose.location.z
        init_dict["ego_state"]["pitch"] = start_pose.rotation.pitch
        init_dict["ego_state"]["roll"] = start_pose.rotation.roll
        init_dict["ego_state"]["yaw"] = start_pose.rotation.yaw

        print('Waiting for CARMA to connect')
        conn, addr = serv.accept()
        print('Connection established with CARMA client')
        
        is_carma_ready = False
        while is_carma_ready is False:
            print('Waiting for CARMA to be ready')

            clock.tick()
            world.tick()
            world_snapshot = world.get_snapshot()
            timestamp = world_snapshot.timestamp
            init_dict["timestamp"] = timestamp.elapsed_seconds

            carma_init_message = conn.recv(4096)
            msg = carma_init_message.decode('utf-8')
            carma_init_dict = json.loads(msg)
            try:
                is_carma_ready = carma_init_dict["isReady"]
            except:
                print('Something is wrong with the message')
            data_tx = json.dumps(init_dict).encode('utf-8')
            conn.sendall(data_tx)
            time.sleep(1)
        print('CARMA starts, sleep 30s for others to launch')
        time.sleep(30)

        dict_veh_list = {k: [] for k in range(len(vehicles_list))}
        ego_veh_data = {}

        while True:
            if should_quit():
                return
            clock.tick()
            world.tick()
            world_snapshot = world.get_snapshot()
            timestamp = world_snapshot.timestamp
            JSON_dict = {}
            JSON_dict["timestamp"] = timestamp.elapsed_seconds
            veh_data = {}
            i = 0
            for vehicle_id in vehicles_list:
                actual_actor = world.get_actor(vehicle_id)
                actor_snapshot = world_snapshot.find(vehicle_id)
                veh_data = parseSnapshot_2_dictionary(id=vehicle_id, snapshot=actor_snapshot, isEgo=False)
                dict_veh_list[i] = veh_data
                i = i+1

            ego_snapshot = world_snapshot.find(ego_vehicle.id)
            ego_veh_data = parseSnapshot_2_dictionary(ego_vehicle.id, ego_snapshot, True)

            JSON_dict["veh_array"] = dict_veh_list
            JSON_dict["ego_state"] = ego_veh_data
            json_to_send = {}
            try:
                json_to_send = json.dumps(JSON_dict)
                data = conn.recv(4096)
                data_tx = json.dumps(json_to_send).encode('utf-8')
                conn.sendall(data_tx)
                print(data)
                rx_msg = data
                CARMA_control = json.loads(rx_msg.decode('utf-8'))
                
            except Exception as e:
                print('Could not generate JSON', e)

            ego_vehicle.apply_control(carla.VehicleControl(throttle=CARMA_control["throttle"],
                                                           steer=CARMA_control["steering"],
                                                           brake=CARMA_control["brake"]))
            i = i+1
            draw_image(display, array_image)
            display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
            pygame.display.flip()



    finally:
        # if args.sync and synchronous_master:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        carla.command.DestroyActor(camera_rgb.id)
        carla.command.DestroyActor(ego_vehicle.id)
        pygame.quit()
        #time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
