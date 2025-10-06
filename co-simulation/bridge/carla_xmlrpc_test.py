#!/usr/bin/env python

# Copyright (c) 2025 MSC Lab, University of Georgia. All rights reserved.

# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.

# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0

# SPDX-License-Identifier: EPL-2.0

# Contact: Zongtan.Li@uga.edu
"""
Improved CARLA XML-RPC end-to-end test.

Enhancements vs original:
- Robust spawn: tries multiple vehicle blueprints until success.
- Validates set_actor_state_properties by checking transform changes.
- Prints sensor payload size (bytes) and essential metadata.
- Waits/retries for sensor frames with timeout.
- Pretty, structured output and explicit pass/fail markers.
- Accepts --url for custom server endpoint (default http://localhost:8090).

Requires: A running CARLA server and the XML-RPC bridge server.
"""
# run with python3 /mnt/data/carla_xmlrpc_test_plus.py --url http://localhost:8090
import argparse
import json
import time
import xmlrpc.client
from typing import Any, Dict, Iterable, Optional

def pretty(obj: Any) -> str:
    return json.dumps(obj, indent=2, ensure_ascii=False)

def section(title: str):
    print("\n" + "=" * 10 + f" {title} " + "=" * 10)

def status(ok: bool, msg: str = ""):
    print(("[PASS] " if ok else "[FAIL] ") + msg)

def try_first(server, methods: Iterable[str]) -> Optional[str]:
    for m in methods:
        try:
            ok = bool(server.spawn_actor(m, "test_car", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], {"role_name": "autopilot"}))
        except Exception:
            ok = False
        if ok:
            return m
    return None

def get_actor_id_list(server, pattern="vehicle.*"):
    try:
        return list(server.get_active_actor_ids(pattern) or [])
    except Exception:
        return []

def wait_sensor(server, key: str, timeout_s: float = 5.0, poll_hz: float = 10.0) -> Optional[Dict[str, Any]]:
    t0 = time.time()
    dt = 1.0 / poll_hz
    while time.time() - t0 < timeout_s:
        data = server.get_sensor_data(key)
        if data and isinstance(data, dict) and data.get("data_blob"):
            return data
        time.sleep(dt)
    return server.get_sensor_data(key)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8090", help="XML-RPC server URL")
    args = ap.parse_args()

    server = xmlrpc.client.ServerProxy(args.url, allow_none=True)

    section("Connection")
    ok = bool(server.connect())
    print("connect():", ok)
    status(ok, "connected")
    # Default to CARLA input frame for this test to avoid SUMO transform
    try:
        print("set_input_frame_mode('carla'):", server.set_input_frame_mode('carla'))
    except Exception as e:
        print("set_input_frame_mode failed:", e)
    print("is_connected():", server.is_connected())

    section("Simulation Control")
    print("advance_simulation():", server.advance_simulation())
    print("step_simulation(0.1):", server.step_simulation(0.1))
    print("get_simulation_time():", server.get_simulation_time())

    section("Map Functions")
    mname = server.get_map_name()
    print("get_map_name():", mname)
    print("get_available_maps():", pretty(server.get_available_maps()))
    # if mname:
        # Do not force reload if it fails; just report.
        # print("load_map(curr_map):", server.load_map("Town02"))

    section("Actor Lifecycle (robust spawn)")
    # Try a few common blueprints to maximize spawn success
    candidates = [
        "vehicle.tesla.model3",
        "vehicle.audi.tt",
        "vehicle.lincoln.mkz_2017",
        "vehicle.nissan.micra",
        "vehicle.bmw.grandtourer",
    ]
    chosen = try_first(server, candidates)
    print("spawn_actor():", bool(chosen), "| blueprint:", chosen)
    try:
        print("set_spectator_to_actor('test_car','follow'):", server.set_spectator_to_actor('test_car', 'follow', 12.0, 6.0, -18.0))
    except Exception as e:
        print("set_spectator_to_actor failed:", e)
    print("get_all_actors():", pretty(server.get_all_actors()))
    before_ids = set(get_actor_id_list(server))

    # Move and set velocity only if we spawned
    if chosen:
        # Focus spectator to our spawned actor alias 'test_car'

        ok_t = server.update_actor_transform("test_car", [0.2, 0.1, 0.8], [100.0, 200.0, 0.0])
        ok_v = server.update_actor_velocity("test_car", [10.0, 0.0, 0.0])
        print("update_actor_transform():", ok_t)
        print("update_actor_velocity():", ok_v)
    else:
        print("update_actor_* skipped (no actor)")
        

    section("Actor Data + state update verification")
    ids = get_actor_id_list(server, "vehicle.*")
    print("get_active_actor_ids('vehicle.*'):", ids)
    # Pick one ID: prefer our spawned actor if any (by querying basic info for "test_car")
    target_id = None
    if ids:
        target_id = ids[0]
    if ids and chosen:
        # Find the actor id that matches alias "test_car" by checking transforms before/after a nudge
        # (Server does not expose alias->id directly, so we just use the first vehicle id.)
        pass

    if target_id is not None:
        print("get_actor_basic_info():", pretty(server.get_actor_basic_info(target_id)))
        t0 = server.get_actor_transform(target_id)
        print("get_actor_transform() [before]:", pretty(t0))
        print("get_actor_velocity():", pretty(server.get_actor_velocity(target_id)))
        print("get_actor_acceleration():", pretty(server.get_actor_acceleration(target_id)))
        print("get_actor_angular_velocity():", pretty(server.get_actor_angular_velocity(target_id)))
        print("get_actor_bounding_box():", pretty(server.get_actor_bounding_box(target_id)))
        print("get_vehicle_light_state():", pretty(server.get_vehicle_light_state(target_id)))

        # State update roundtrip check: move by +3m in x
        ok_set = server.set_actor_state_properties(target_id, {
            "transform": {"location": {"x": (t0 or {}).get("location", {}).get("x", 0.0) + 3.0,
                                       "y": (t0 or {}).get("location", {}).get("y", 0.0),
                                       "z": (t0 or {}).get("location", {}).get("z", 0.0)},
                          "rotation": {"pitch": 0, "yaw": 0, "roll": 0}},
            "target_velocity": {"x": 2, "y": 0, "z": 0}
        })
        print("set_actor_state_properties():", ok_set)
        time.sleep(0.2)
        t1 = server.get_actor_transform(target_id)
        print("get_actor_transform() [after]:", pretty(t1))
        moved = False
        try:
            if t0 and t1:
                moved = (t1["location"]["x"] - t0["location"]["x"]) > 1.0
        except Exception:
            moved = False
        status(moved, "transform updated by > 1m in x")
    else:
        print("No vehicles in world; skipping actor data verification.")

    section("Traffic Lights")
    tls = server.get_all_traffic_light_states()
    print("get_all_traffic_light_states():", pretty(tls[:10] if isinstance(tls, list) else tls))
    if isinstance(tls, list) and len(tls) > 0:
        tid = tls[0].get("id")
        if tid is not None:
            print("get_traffic_light_state():", pretty(server.get_traffic_light_state(tid)))
            print("set_traffic_light_state(Red):", server.set_traffic_light_state(tid, "Red"))
            print("set_traffic_light_timer(5.0):", server.set_traffic_light_timer(tid, 5.0))

    section("Sensor Tests (payload size & metadata)")
    created = server.create_sensor(
        "sensor.camera.rgb", "test_cam",
        [0.0, 0.0, 3.0], [0.0, 0.0, 0.0],
        {"image_size_x": 640, "image_size_y": 480, "fov": 90}
    )
    print("create_sensor(camera.rgb):", created)
    data = None
    if created:
        data = wait_sensor(server, "test_cam", timeout_s=5.0, poll_hz=10.0)
        if data:
            try:
                # xmlrpc.client.Binary exposes .data (bytes)
                blob = data.get("data_blob")
                size = len(blob.data) if hasattr(blob, "data") and blob is not None else 0
            except Exception:
                size = 0
            print("get_sensor_data(test_cam): size(bytes) =", size)
            meta = data.get("metadata", {})
            keep = {k: meta.get(k) for k in ["width", "height", "fov", "image_format"] if k in meta}
            print("metadata:", pretty(keep if keep else meta))
            status(size > 0, "received at least one sensor frame")
        else:
            print("get_sensor_data(test_cam): None")
    # time.sleep(50)  # Wait a bit before destroying sensor
    print("destroy_sensor(test_cam):", server.destroy_sensor("test_cam"))

    section("Destroy Actor / Disconnect")
    print("destroy_actor(test_car):", server.destroy_actor("test_car"))
    print("disconnect():", server.disconnect())
    print("is_connected():", server.is_connected())

if __name__ == "__main__":
    main()
