import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list=[]

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    map = world.get_map()


    landmark_list = map.get_all_landmarks_of_type('1000001')
    for item in landmark_list:
        print(item.id, " ", item,type)


    for landmark in landmark_list:
        world.debug.draw_string(landmark.transform.location, str(landmark.id), draw_shadow=False,
											 color=carla.Color(r=255, g=0, b=0), life_time=200,
											 persistent_lines=True)



finally:
    # print('Cleaning up actors...')
    # for actor in actor_list:
    #     actor.destroy()
    print('Done!')