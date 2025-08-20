#!/usr/bin/env python3
import xmlrpc.client
import json
import time


def pretty(obj):
    return json.dumps(obj, indent=2)


def main():
    server = xmlrpc.client.ServerProxy("http://localhost:8090", allow_none=True)

    print("===== Connection Tests =====")
    print("connect():", server.connect())
    print("is_connected():", server.is_connected())

    print("\n===== Simulation Control =====")
    print("advance_simulation():", server.advance_simulation())
    print("step_simulation(0.1):", server.step_simulation(0.1))
    print("get_simulation_time():", server.get_simulation_time())

    print("\n===== Map Functions =====")
    print("get_map_name():", server.get_map_name())
    print("get_available_maps():", server.get_available_maps())
    # 尝试 reload 当前 map
    curr_map = server.get_map_name()
    if curr_map:
        print("load_map(curr_map):", server.load_map(curr_map))

    print("\n===== Actor Lifecycle =====")
    # spawn 一个车辆
    print("spawn_actor():", server.spawn_actor(
        "vehicle.tesla.model3", "test_car",
        [0.0, 0.0, 2.0],  # location (x,y,z)
        [0.0, 0.0, 0.0],  # rotation (pitch,yaw,roll)
        {"role_name": "autopilot"}
    ))
    time.sleep(1)
    print("get_all_actors():", pretty(server.get_all_actors()))

    print("update_actor_transform():", server.update_actor_transform("test_car",
        [5.0, 0.0, 2.0], [0.0, 0.0, 0.0]))
    print("update_actor_velocity():", server.update_actor_velocity("test_car", [10.0, 0.0, 0.0]))

    print("\n===== Actor Data =====")
    ids = server.get_active_actor_ids("vehicle.*")
    print("get_active_actor_ids('vehicle.*'):", ids)
    if ids:
        aid = ids[0]
        print("get_actor_basic_info():", pretty(server.get_actor_basic_info(aid)))
        print("get_actor_transform():", pretty(server.get_actor_transform(aid)))
        print("get_actor_velocity():", pretty(server.get_actor_velocity(aid)))
        print("get_actor_acceleration():", pretty(server.get_actor_acceleration(aid)))
        print("get_actor_angular_velocity():", pretty(server.get_actor_angular_velocity(aid)))
        print("get_actor_bounding_box():", pretty(server.get_actor_bounding_box(aid)))
        print("get_vehicle_light_state():", pretty(server.get_vehicle_light_state(aid)))

        print("set_actor_state_properties():", server.set_actor_state_properties(
            aid,
            {
                "transform": {"location": {"x": 10, "y": 0, "z": 2},
                              "rotation": {"pitch": 0, "yaw": 0, "roll": 0}},
                "target_velocity": {"x": 5, "y": 0, "z": 0},
                "control": {"throttle": 0.5, "steer": 0.0, "brake": 0.0}
            }
        ))

    print("\n===== Traffic Lights =====")
    tls = server.get_all_traffic_light_states()
    print("get_all_traffic_light_states():", pretty(tls))
    if tls:
        tid = tls[0]["id"]
        print("get_traffic_light_state():", pretty(server.get_traffic_light_state(tid)))
        print("set_traffic_light_state(Red):", server.set_traffic_light_state(tid, "Red"))
        print("set_traffic_light_timer(5.0):", server.set_traffic_light_timer(tid, 5.0))

    print("\n===== Sensor Tests =====")
    print("create_sensor(camera.rgb):", server.create_sensor(
        "sensor.camera.rgb", "test_cam",
        [0.0, 0.0, 3.0], [0.0, 0.0, 0.0],
        {"image_size_x": 800, "image_size_y": 600, "fov": 90}
    ))
    time.sleep(2)
    data = server.get_sensor_data("test_cam")
    print("get_sensor_data(test_cam):", "OK" if data else "None")
    print("destroy_sensor(test_cam):", server.destroy_sensor("test_cam"))

    print("\n===== Destroy Actor =====")
    print("destroy_actor(test_car):", server.destroy_actor("test_car"))

    print("\n===== Disconnect =====")
    print("disconnect():", server.disconnect())
    print("is_connected():", server.is_connected())


if __name__ == "__main__":
    main()
