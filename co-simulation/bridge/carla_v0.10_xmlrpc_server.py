#!/usr/bin/env python
# Copyright (c) 2025 University of Georgia. All rights reserved.

# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.

# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0

# SPDX-License-Identifier: EPL-2.0
# Contact: Zongtan.Li@uga.edu

try:
    CARLA_VERSION = getattr(carla, "__version__", "unknown")
except Exception:
    CARLA_VERSION = "unknown"

"""
CARLA XML-RPC Server for MOSAIC Integration (Unified XML-RPC per redesign spec)

This server exposes granular, XML-RPC-safe methods for:
- Actor data (vehicles, pedestrians): transforms, velocities, accelerations, bounding boxes, etc.
- Traffic signal states (single and bulk).
- Sensor frames (raw bytes + metadata) with xmlrpc.client.Binary.
- Simulation control with world.tick() via advance_simulation().

It aligns with the "Proposed Redesign of the CARLA-MOSAIC Bridge Using Unified XML-RPC"
specification (UGA MSC Lab, 2025-06-03), Sections 4–6.
"""

import argparse
import logging
import sys
import os
import json
import threading
from typing import Dict, List, Optional, Tuple, Any, Union
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
from xmlrpc.client import Binary
import glob
import time



import carla

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("CarlaXMLRPCServer")

ActorKey = Union[int, str]
SensorKey = Union[int, str]


class CarlaXMLRPCServer:
    def __init__(self, host: str = 'localhost', port: int = 8090,
                 carla_host: str = 'localhost', carla_port: int = 2000):
        self.host = host
        self.port = port
        self.carla_host = carla_host
        self.carla_port = carla_port

        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None

        self.actors: Dict[str, carla.Actor] = {}
        self.actor_types: Dict[str, str] = {}
        self.actor_blueprints: Dict[str, carla.ActorBlueprint] = {}

        self.sensors: Dict[str, carla.Sensor] = {}
        self.sensor_data: Dict[str, Any] = {}
        self.sensor_blueprints: Dict[str, carla.ActorBlueprint] = {}

        self.lock = threading.RLock()

        self.server = SimpleXMLRPCServer(
            (host, port),
            requestHandler=SimpleXMLRPCRequestHandler,
            allow_none=True, logRequests=False
        )
        self._register_methods()
        logger.info("XML-RPC methods registered")

    # ---------- Registration ----------
    def _register_methods(self):
        # Connection
        self.server.register_function(self.connect, 'connect')
        self.server.register_function(self.disconnect, 'disconnect')
        self.server.register_function(self.is_connected, 'is_connected')

        # Simulation control
        self.server.register_function(self.advance_simulation, 'advance_simulation')
        self.server.register_function(self.step_simulation, 'step_simulation')  # backward compat
        self.server.register_function(self.get_simulation_time, 'get_simulation_time')

        # Actor discovery & getters
        self.server.register_function(self.get_active_actor_ids, 'get_active_actor_ids')
        self.server.register_function(self.get_actor_basic_info, 'get_actor_basic_info')
        self.server.register_function(self.get_actor_transform, 'get_actor_transform')
        self.server.register_function(self.get_actor_velocity, 'get_actor_velocity')
        self.server.register_function(self.get_actor_acceleration, 'get_actor_acceleration')
        self.server.register_function(self.get_actor_angular_velocity, 'get_actor_angular_velocity')
        self.server.register_function(self.get_actor_bounding_box, 'get_actor_bounding_box')
        self.server.register_function(self.get_vehicle_light_state, 'get_vehicle_light_state')

        # Writable
        self.server.register_function(self.set_actor_state_properties, 'set_actor_state_properties')

        # Lifecycle / utility
        self.server.register_function(self.spawn_actor, 'spawn_actor')
        self.server.register_function(self.destroy_actor, 'destroy_actor')
        self.server.register_function(self.update_actor_transform, 'update_actor_transform')
        self.server.register_function(self.update_actor_velocity, 'update_actor_velocity')
        self.server.register_function(self.get_all_actors, 'get_all_actors')

        # Traffic lights
        self.server.register_function(self.get_traffic_light_state, 'get_traffic_light_state')
        self.server.register_function(self.get_all_traffic_light_states, 'get_all_traffic_light_states')
        self.server.register_function(self.set_traffic_light_state, 'set_traffic_light_state')
        self.server.register_function(self.set_traffic_light_timer, 'set_traffic_light_timer')

        # Sensors
        self.server.register_function(self.create_sensor, 'create_sensor')
        self.server.register_function(self.destroy_sensor, 'destroy_sensor')
        self.server.register_function(self.get_sensor_data, 'get_sensor_data')
        self.server.register_function(self.get_detected_objects, 'get_detected_objects')

        # Maps
        self.server.register_function(self.get_map_name, 'get_map_name')
        self.server.register_function(self.get_available_maps, 'get_available_maps')
        self.server.register_function(self.load_map, 'load_map')

    # ---------- Utilities ----------
    def _sim_timestamp(self) -> float:
        try:
            if self.world is None:
                return 0.0
            snap = self.world.get_snapshot()
            if snap is None:
                return 0.0
            return float(snap.timestamp.elapsed_seconds)
        except Exception:
            return 0.0

    def _resolve_actor(self, actor_key: ActorKey) -> Optional[carla.Actor]:
        if isinstance(actor_key, str):
            if actor_key in self.actors:
                return self.actors[actor_key]
            try:
                aid = int(actor_key)
                if self.world:
                    return self.world.get_actor(aid)
            except Exception:
                return None
        else:
            try:
                if self.world:
                    return self.world.get_actor(int(actor_key))
            except Exception:
                return None
        return None

    def _resolve_sensor(self, sensor_key: SensorKey) -> Tuple[Optional[str], Optional[carla.Sensor]]:
        if isinstance(sensor_key, str):
            if sensor_key in self.sensors:
                return sensor_key, self.sensors[sensor_key]
            try:
                sid = int(sensor_key)
                if self.world:
                    s = self.world.get_actor(sid)
                    for alias, ss in self.sensors.items():
                        if ss.id == sid:
                            return alias, ss
                    return str(sid), s
            except Exception:
                return None, None
        else:
            sid = int(sensor_key)
            if self.world:
                s = self.world.get_actor(sid)
                for alias, ss in self.sensors.items():
                    if ss.id == sid:
                        return alias, ss
                return str(sid), s
        return None, None

    # ---------- Connection ----------
    def connect(self) -> bool:
        try:
            with self.lock:
                if self.client is None:
                    self.client = carla.Client(self.carla_host, self.carla_port)
                    self.client.set_timeout(10.0)
                self.world = self.client.get_world()
                try:
                    current_map = self.world.get_map().name
                except Exception:
                    current_map = "<unknown>"
                logger.info("Connected to CARLA %s at %s:%s | map=%s",
                            CARLA_VERSION, self.carla_host, self.carla_port, current_map)
                return True
        except Exception as e:
            logger.error("Failed to connect: %s", e)
            return False

    def disconnect(self) -> bool:
        try:
            with self.lock:
                for _, a in list(self.actors.items()):
                    try: a.destroy()
                    except Exception: pass
                for _, s in list(self.sensors.items()):
                    try: s.destroy()
                    except Exception: pass
                self.actors.clear(); self.actor_types.clear(); self.actor_blueprints.clear()
                self.sensors.clear(); self.sensor_data.clear(); self.sensor_blueprints.clear()
                self.world = None; self.client = None
                logger.info("Disconnected from CARLA")
                return True
        except Exception as e:
            logger.error("Error during disconnect: %s", e)
            return False

    def is_connected(self) -> bool:
        return self.client is not None and self.world is not None

    # ---------- Simulation ----------
    def advance_simulation(self) -> bool:
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                self.world.tick()
                return True
        except Exception as e:
            logger.error("advance_simulation error: %s", e)
            return False

    def step_simulation(self, delta_time: float) -> bool:
        # Kept for backward compatibility; no-op in unified design
        return self.is_connected()

    def get_simulation_time(self) -> float:
        return self._sim_timestamp()

    # ---------- Actor Lifecycle ----------
    def spawn_actor(self, actor_type: str, actor_id: str,
                    location: List[float], rotation: List[float],
                    attributes: Dict[str, Any] = None) -> bool:
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                if actor_id in self.actors:
                    return False

                lib = self.world.get_blueprint_library()

                bp = None
                req = (actor_type or "").strip()

                if req:
                    try:
                        bp = lib.find(req)
                    except Exception:
                        bp = None

                if bp is None and req:
                    cands = lib.filter(req)
                    if cands:
                        bp = cands[0]

                if bp is None:
                    for pattern in ("vehicle.*", "walker.pedestrian.*", "*"):
                        cands = lib.filter(pattern)
                        if cands:
                            bp = cands[0]
                            break

                if bp is None:
                    logger.error("No blueprint available for actor_type=%r", actor_type)
                    return False

                if attributes:
                    for k, v in attributes.items():
                        if bp.has_attribute(k):
                            bp.set_attribute(k, str(v))

                transform = carla.Transform(
                    carla.Location(*[float(v) for v in location]),
                    carla.Rotation(*[float(v) for v in rotation])
                )

                actor = self.world.spawn_actor(bp, transform)
                self.actors[actor_id] = actor
                self.actor_types[actor_id] = getattr(bp, "id", req) or req
                self.actor_blueprints[actor_id] = bp
                return True
        except Exception as e:
            logger.error("spawn_actor error: %s", e)
            return False


    def destroy_actor(self, actor_key: ActorKey) -> bool:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return False
                for alias, a in list(self.actors.items()):
                    if a.id == actor.id:
                        self.actors.pop(alias, None)
                        self.actor_types.pop(alias, None)
                        self.actor_blueprints.pop(alias, None)
                actor.destroy()
                return True
        except Exception as e:
            logger.error("destroy_actor error: %s", e)
            return False

    def update_actor_transform(self, actor_key: ActorKey, location: List[float], rotation: List[float]) -> bool:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return False
                transform = carla.Transform(
                    carla.Location(*[float(v) for v in location]),
                    carla.Rotation(*[float(v) for v in rotation])
                )
                actor.set_transform(transform)
                return True
        except Exception as e:
            logger.error("update_actor_transform error: %s", e)
            return False

    def update_actor_velocity(self, actor_key: ActorKey, velocity: List[float]) -> bool:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None:
                    return False

                if isinstance(velocity, dict):
                    vx = float(velocity.get('x', 0.0))
                    vy = float(velocity.get('y', 0.0))
                    vz = float(velocity.get('z', 0.0))
                else:
                    if not hasattr(velocity, '__len__') or len(velocity) != 3:
                        logger.error("update_actor_velocity expects length-3 sequence or dict {x,y,z}")
                        return False
                    vx, vy, vz = (float(velocity[0]), float(velocity[1]), float(velocity[2]))

                vec = carla.Vector3D(vx, vy, vz)

                if hasattr(actor, 'set_target_velocity'):
                    actor.set_target_velocity(vec)
                    return True

                if hasattr(actor, 'set_velocity'):
                    try:
                        if hasattr(actor, 'set_simulate_physics'):
                            try:
                                actor.set_simulate_physics(True)
                            except Exception:
                                pass
                        actor.set_velocity(vec)
                        return True
                    except Exception as e:
                        logger.debug("set_velocity failed: %s", e)
                        return False

                return False
        except Exception as e:
            logger.error("update_actor_velocity error: %s", e)
            return False


    def get_all_actors(self) -> Dict[str, Dict[str, Any]]:
        try:
            with self.lock:
                out = {}
                for alias, actor in self.actors.items():
                    t = actor.get_transform()
                    out[alias] = {
                        'type': self.actor_types.get(alias, getattr(actor, 'type_id', '')),
                        'location': [float(t.location.x), float(t.location.y), float(t.location.z)],
                        'rotation': [float(t.rotation.pitch), float(t.rotation.yaw), float(t.rotation.roll)]
                    }
                return out
        except Exception as e:
            logger.error("get_all_actors error: %s", e)
            return {}

    # ---------- Actor Data (spec) ----------
    def get_active_actor_ids(self, filter_pattern: str = "vehicle.*") -> List[int]:
        try:
            with self.lock:
                if not self.is_connected(): return []
                return [int(a.id) for a in self.world.get_actors().filter(filter_pattern)]
        except Exception as e:
            logger.error("get_active_actor_ids error: %s", e)
            return []

    def get_actor_basic_info(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return None
                return {
                    'actor_id': int(actor.id),
                    'type_id': str(actor.type_id),
                    'status': "Active" if bool(getattr(actor, 'is_alive', True)) else "Invalid",
                    'is_alive': bool(getattr(actor, 'is_alive', True)),
                    'timestamp': self._sim_timestamp()
                }
        except Exception as e:
            logger.error("get_actor_basic_info error: %s", e)
            return None

    def get_actor_transform(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return None
                t = actor.get_transform()
                return {
                    'location': {'x': float(t.location.x), 'y': float(t.location.y), 'z': float(t.location.z)},
                    'rotation': {'pitch': float(t.rotation.pitch), 'yaw': float(t.rotation.yaw), 'roll': float(t.rotation.roll)},
                    'timestamp': self._sim_timestamp()
                }
        except Exception as e:
            logger.error("get_actor_transform error: %s", e)
            return None

    def get_actor_velocity(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_velocity'): return None
                v = actor.get_velocity()
                return {'x': float(v.x), 'y': float(v.y), 'z': float(v.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_velocity error: %s", e)
            return None

    def get_actor_acceleration(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_acceleration'): return None
                a = actor.get_acceleration()
                return {'x': float(a.x), 'y': float(a.y), 'z': float(a.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_acceleration error: %s", e)
            return None

    def get_actor_angular_velocity(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_angular_velocity'): return None
                w = actor.get_angular_velocity()
                return {'x': float(w.x), 'y': float(w.y), 'z': float(w.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_angular_velocity error: %s", e)
            return None

    def get_actor_bounding_box(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'bounding_box'): return None
                bb = actor.bounding_box
                return {
                    'extent': {'x': float(bb.extent.x), 'y': float(bb.extent.y), 'z': float(bb.extent.z)},
                    'location_offset': {'x': float(bb.location.x), 'y': float(bb.location.y), 'z': float(bb.location.z)},
                    'timestamp': self._sim_timestamp()
                }
        except Exception as e:
            logger.error("get_actor_bounding_box error: %s", e)
            return None

    def get_vehicle_light_state(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return None
                if hasattr(actor, 'get_light_state'):
                    st = actor.get_light_state()
                    return {'light_state_int': int(st), 'timestamp': self._sim_timestamp()}
                return None
        except Exception as e:
            logger.error("get_vehicle_light_state error: %s", e)
            return None

    def set_actor_state_properties(self, actor_key: ActorKey, properties_to_set: Dict[str, Any]) -> bool:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return False

                t_in = properties_to_set.get('transform')
                if t_in:
                    loc = t_in.get('location', {}); rot = t_in.get('rotation', {})
                    transform = carla.Transform(
                        carla.Location(float(loc.get('x', 0.0)), float(loc.get('y', 0.0)), float(loc.get('z', 0.0))),
                        carla.Rotation(float(rot.get('pitch', 0.0)), float(rot.get('yaw', 0.0)), float(rot.get('roll', 0.0)))
                    )
                    actor.set_transform(transform)

                tv = properties_to_set.get('target_velocity')
                if tv:
                    vec = carla.Vector3D(float(tv.get('x', 0.0)), float(tv.get('y', 0.0)), float(tv.get('z', 0.0)))
                    if hasattr(actor, 'set_target_velocity'):
                        actor.set_target_velocity(vec)
                    elif hasattr(actor, 'set_velocity'):
                        actor.set_velocity(vec)

                tav = properties_to_set.get('target_angular_velocity')
                if tav:
                    avec = carla.Vector3D(float(tav.get('x', 0.0)), float(tav.get('y', 0.0)), float(tav.get('z', 0.0)))
                    if hasattr(actor, 'set_target_angular_velocity'):
                        actor.set_target_angular_velocity(avec)
                    elif hasattr(actor, 'set_angular_velocity'):
                        actor.set_angular_velocity(avec)
                ctrl = properties_to_set.get('control')  # {'throttle':..,'steer':..,'brake':..,'reverse':..}
            if ctrl and str(getattr(actor, 'type_id', '')).startswith('vehicle.'):
                try:
                    c = carla.VehicleControl()
                    for k, v in ctrl.items():
                        if hasattr(c, k):
                            setattr(c, k, v)
                    actor.apply_control(c)
                except Exception as e:
                    logger.debug("apply_control ignored: %s", e)

                return True
        except Exception as e:
            logger.error("set_actor_state_properties error: %s", e)
            return False

    # ---------- Traffic Lights ----------
    def _tl_state_to_int(self, state: carla.TrafficLightState) -> int:
        if state == carla.TrafficLightState.Red: return 0
        if state == carla.TrafficLightState.Yellow: return 1
        if state == carla.TrafficLightState.Green: return 2
        if state == carla.TrafficLightState.Off: return 3
        return 4

    def get_traffic_light_state(self, traffic_light_id: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                if not self.is_connected(): return None
                tid = int(traffic_light_id)
                tl = self.world.get_actor(tid) if self.world else None
                if tl is None or 'traffic_light' not in getattr(tl, 'type_id', ''): return None
                state = tl.get_state()
                data = {
                    'id': int(tl.id),
                    'state': int(self._tl_state_to_int(state)),
                    'elapsed_time': float(getattr(tl, 'get_elapsed_time', lambda: 0.0)()),
                    'timestamp': self._sim_timestamp()
                }
                try: data['is_frozen'] = bool(tl.is_frozen())
                except Exception: pass
                try: data['pole_index'] = int(tl.get_pole_index())
                except Exception: pass
                return data
        except Exception as e:
            logger.error("get_traffic_light_state error: %s", e)
            return None

    def get_all_traffic_light_states(self) -> List[Dict[str, Any]]:
        try:
            with self.lock:
                if not self.is_connected(): return []
                out = []
                ts = self._sim_timestamp()
                for tl in self.world.get_actors().filter('traffic.traffic_light'):
                    try:
                        state = tl.get_state()
                        item = {
                            'id': int(tl.id),
                            'state': int(self._tl_state_to_int(state)),
                            'elapsed_time': float(getattr(tl, 'get_elapsed_time', lambda: 0.0)()),
                            'timestamp': ts
                        }
                        try: item['is_frozen'] = bool(tl.is_frozen())
                        except Exception: pass
                        try: item['pole_index'] = int(tl.get_pole_index())
                        except Exception: pass
                        out.append(item)
                    except Exception:
                        continue
                return out
        except Exception as e:
            logger.error("get_all_traffic_light_states error: %s", e)
            return []

    def set_traffic_light_state(self, traffic_light_id: ActorKey, state: str) -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                for tl in self.world.get_actors().filter('traffic.traffic_light'):
                    if str(tl.id) == str(traffic_light_id):
                        if state == 'Red':
                            tl.set_state(carla.TrafficLightState.Red)
                        elif state == 'Yellow':
                            tl.set_state(carla.TrafficLightState.Yellow)
                        elif state == 'Green':
                            tl.set_state(carla.TrafficLightState.Green)
                        else:
                            return False
                        return True
                return False
        except Exception as e:
            logger.error("set_traffic_light_state error: %s", e)
            return False

    def set_traffic_light_timer(self, traffic_light_id: ActorKey, time_s: float) -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                for tl in self.world.get_actors().filter('traffic.traffic_light'):
                    if str(tl.id) == str(traffic_light_id):
                        tl.set_green_time(float(time_s))
                        return True
                return False
        except Exception as e:
            logger.error("set_traffic_light_timer error: %s", e)
            return False

    # ---------- Sensors ----------
    def create_sensor(self, sensor_type: str, sensor_id: str,
                      location: List[float], rotation: List[float],
                      attributes: Dict[str, Any] = None) -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                if sensor_id in self.sensors: return False
                bp = self.world.get_blueprint_library().find(sensor_type)
                if not bp: return False
                if attributes:
                    for k, v in attributes.items():
                        if bp.has_attribute(k): bp.set_attribute(k, str(v))
                transform = carla.Transform(
                    carla.Location(*[float(v) for v in location]),
                    carla.Rotation(*[float(v) for v in rotation])
                )
                sensor = self.world.spawn_actor(bp, transform)
                sensor.listen(lambda data, sid=sensor_id: self._sensor_callback(sid, data))
                self.sensors[sensor_id] = sensor
                self.sensor_data[sensor_id] = None
                self.sensor_blueprints[sensor_id] = bp
                return True
        except Exception as e:
            logger.error("create_sensor error: %s", e)
            return False

    def destroy_sensor(self, sensor_key: SensorKey) -> bool:
        try:
            with self.lock:
                alias, sensor = self._resolve_sensor(sensor_key)
                if sensor is None: return False
                sensor.destroy()
                if alias is not None:
                    self.sensors.pop(alias, None)
                    self.sensor_data.pop(alias, None)
                    self.sensor_blueprints.pop(alias, None)
                return True
        except Exception as e:
            logger.error("destroy_sensor error: %s", e)
            return False

    def _sensor_callback(self, alias_key: str, data: Any):
        try:
            with self.lock:
                sensor = self.sensors.get(alias_key, None)
                if sensor is None: return
                out: Dict[str, Any] = {
                    'sensor_id': int(getattr(sensor, 'id', -1)),
                    'sensor_type': str(getattr(sensor, 'type_id', '')),
                    'frame': int(getattr(data, 'frame', 0)),
                    'timestamp': float(getattr(data, 'timestamp', self._sim_timestamp())),
                }
                # Transform at measurement
                try:
                    t = getattr(data, 'transform', None) or sensor.get_transform()
                    out['transform_at_measurement'] = {
                        'location': {'x': float(t.location.x), 'y': float(t.location.y), 'z': float(t.location.z)},
                        'rotation': {'pitch': float(t.rotation.pitch), 'yaw': float(t.rotation.yaw), 'roll': float(t.rotation.roll)},
                        'timestamp': self._sim_timestamp()
                    }
                except Exception:
                    pass

                # Metadata from blueprint
                meta: Dict[str, Any] = {}
                bp = self.sensor_blueprints.get(alias_key, None)
                if bp is not None:
                    try:
                        for attr in bp:
                            try:
                                meta[attr.id] = attr.as_string()
                            except Exception:
                                meta[attr.id] = str(attr)
                    except Exception:
                        pass

                # Camera (Image)
                if hasattr(data, 'raw_data') and hasattr(data, 'width') and hasattr(data, 'height'):
                    out['data_blob'] = Binary(bytes(getattr(data, 'raw_data', b'')))
                    meta.setdefault('width', int(getattr(data, 'width', 0)))
                    meta.setdefault('height', int(getattr(data, 'height', 0)))
                    # fov may be in attributes
                    try:
                        meta.setdefault('fov', float(sensor.attributes.get('fov')))  # type: ignore
                    except Exception:
                        pass
                    meta.setdefault('image_format', 'BGRA')
                    out['metadata'] = meta

                # LiDAR（v0.10 图像/点云 API 保持一致；此处逻辑不变）
                elif hasattr(data, 'raw_data') or hasattr(data, 'points'):
                    if hasattr(data, 'raw_data'):
                        out['data_blob'] = Binary(bytes(getattr(data, 'raw_data', b'')))
                    else:
                        try:
                            pts_json = json.dumps(getattr(data, 'points', []))
                            out['data_blob'] = Binary(pts_json.encode('utf-8'))
                        except Exception:
                            out['data_blob'] = Binary(b'')
                    out['metadata'] = meta

                # IMU/GNSS/others (fallback)
                else:
                    s = str(data)
                    out['data_blob'] = Binary(s.encode('utf-8'))
                    out['metadata'] = meta

                self.sensor_data[alias_key] = out
        except Exception as e:
            logger.error("sensor_callback error: %s", e)
            self.sensor_data[alias_key] = {'error': str(e), 'timestamp': time.time(), 'sensor_id': -1}

    def get_sensor_data(self, sensor_key: SensorKey) -> Optional[Dict[str, Any]]:
        try:
            with self.lock:
                alias, _ = self._resolve_sensor(sensor_key)
                if alias is None or alias not in self.sensor_data: return None
                return self.sensor_data[alias]
        except Exception as e:
            logger.error("get_sensor_data error: %s", e)
            return None

    def get_detected_objects(self, infrastructure_id: str, sensor_key: SensorKey) -> str:
        # Placeholder: return empty JSON array
        return json.dumps([])

    # ---------- Maps ----------
    def get_map_name(self) -> str:
        try:
            with self.lock:
                if not self.is_connected(): return ""
                return self.world.get_map().name
        except Exception as e:
            logger.error("get_map_name error: %s", e)
            return ""

    def get_available_maps(self) -> List[str]:
        try:
            if not self.client:
                return []
            maps = []
            try:
                # v0.10 仍可能提供此方法，但资源很少（多为 Town10）
                maps = list(self.client.get_available_maps())
            except Exception:
                maps = []
            # v0.10 官方仅保证升级了 Town10，如查询为空则给出兜底提示
            if not maps:
                maps = ['Carla/Maps/Town10HD_Opt', 'Carla/Maps/Town10HD']
            return maps
        except Exception as e:
            logger.error("get_available_maps error: %s", e)
            return []

    def load_map(self, map_name: str) -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                try:
                    self.world = self.client.load_world(map_name)
                    return True
                except Exception as e:
                    # v0.10 缺少大量旧地图；遇到不可用时返回 False 而不是抛异常
                    logger.warning("load_map failed for %s on v0.10: %s", map_name, e)
                    return False
        except Exception as e:
            logger.error("load_map error: %s", e)
            return False

    # ---------- Server lifecycle ----------
    def start(self):
        logger.info("Starting CARLA XML-RPC server on %s:%s", self.host, self.port)
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Server stopped by user")
        finally:
            self.disconnect()

    def stop(self):
        logger.info("Stopping CARLA XML-RPC server")
        try:
            self.server.shutdown()
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(description='CARLA XML-RPC Server (Unified)')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--carla-host', default='localhost')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    server = CarlaXMLRPCServer(args.host, args.port, args.carla_host, args.carla_port)
    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Interrupted")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
