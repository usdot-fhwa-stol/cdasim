#!/usr/bin/env python

# Copyright (c) 2025 MSC Lab, University of Georgia. All rights reserved.

# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.

# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0

# SPDX-License-Identifier: EPL-2.0

# Contact: Zongtan.Li@uga.edu
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

# Add CARLA Python API to path
try:
    sys.path.append(
        glob.glob('PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Cannot find CARLA library .egg file")
    sys.exit(1)

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

        self._odr_to_tl = {} 
        self._odr_to_tls = {}

        self.lock = threading.RLock()

        # External-to-CARLA coordinate transform settings (SUMO/MOSAIC frame → CARLA frame)
        # - input_frame: 'sumo' applies BridgeHelper-like conversion (y inversion, yaw - 90 deg, offset)
        # - net_offset_xy: offset from SUMO net (x, y) applied before handedness flip
        self.input_frame: str = 'sumo'
        self.net_offset_xy: Tuple[float, float] = (0.0, 0.0)

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
        # SUMO-aware spawn/update helpers (apply front-bumper correction like BridgeHelper)
        self.server.register_function(self.spawn_actor_from_sumo, 'spawn_actor_from_sumo')
        self.server.register_function(self.destroy_actor, 'destroy_actor')
        self.server.register_function(self.update_actor_transform, 'update_actor_transform')
        self.server.register_function(self.update_actor_transform_from_sumo, 'update_actor_transform_from_sumo')
        self.server.register_function(self.update_actor_velocity, 'update_actor_velocity')
        self.server.register_function(self.get_all_actors, 'get_all_actors')

        # Coordinate transforms utilities
        self.server.register_function(self.sumo_to_carla_transform, 'sumo_to_carla_transform')
        self.server.register_function(self.carla_to_sumo_transform, 'carla_to_sumo_transform')

        # Traffic lights
        self.server.register_function(self.get_traffic_light_state, 'get_traffic_light_state')
        self.server.register_function(self.get_all_traffic_light_states, 'get_all_traffic_light_states')
        self.server.register_function(self.set_traffic_light_state, 'set_traffic_light_state')
        self.server.register_function(self.set_traffic_light_timer, 'set_traffic_light_timer')
        self.server.register_function(self.freeze_all_traffic_lights, 'freeze_all_traffic_lights')

        # Sensors
        self.server.register_function(self.create_sensor, 'create_sensor')
        self.server.register_function(self.destroy_sensor, 'destroy_sensor')
        self.server.register_function(self.get_sensor_data, 'get_sensor_data')
        self.server.register_function(self.get_detected_objects, 'get_detected_objects')

        # Maps
        self.server.register_function(self.get_map_name, 'get_map_name')
        self.server.register_function(self.get_available_maps, 'get_available_maps')
        self.server.register_function(self.load_map, 'load_map')

        # Coordinate transform configuration
        self.server.register_function(self.set_input_frame_mode, 'set_input_frame_mode')
        self.server.register_function(self.set_net_offset_xy, 'set_net_offset_xy')

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

    # ----- Coordinate transforms (external → CARLA) -----
    def _to_carla_location(self, location_seq: List[float]) -> carla.Location:
        try:
            x_in = float(location_seq[0])
            y_in = float(location_seq[1])
            z_in = float(location_seq[2]) if len(location_seq) > 2 else 0.0
        except Exception:
            x_in, y_in, z_in = 0.0, 0.0, 0.0

        if self.input_frame == 'sumo':
            # Apply SUMO net offset, then convert to CARLA left-handed (invert Y)
            x_off = x_in - float(self.net_offset_xy[0])
            y_off = y_in - float(self.net_offset_xy[1])
            return carla.Location(x_off, -y_off, z_in)
        # Assume already in CARLA world coordinates
        return carla.Location(x_in, y_in, z_in)

    def _to_carla_rotation(self, rotation_seq: List[float]) -> carla.Rotation:
        try:
            pitch_in = float(rotation_seq[0]) if len(rotation_seq) > 0 else 0.0
            yaw_in = float(rotation_seq[1]) if len(rotation_seq) > 1 else 0.0
            roll_in = float(rotation_seq[2]) if len(rotation_seq) > 2 else 0.0
        except Exception:
            pitch_in, yaw_in, roll_in = 0.0, 0.0, 0.0

        if self.input_frame == 'sumo':
            # SUMO → CARLA yaw mapping per BridgeHelper: yaw_carla = yaw_sumo - 90
            return carla.Rotation(pitch_in, yaw_in - 90.0, roll_in)
        return carla.Rotation(pitch_in, yaw_in, roll_in)

    def _to_carla_velocity(self, velocity_seq_or_dict: Any) -> carla.Vector3D:
        if isinstance(velocity_seq_or_dict, dict):
            vx = float(velocity_seq_or_dict.get('x', 0.0))
            vy = float(velocity_seq_or_dict.get('y', 0.0))
            vz = float(velocity_seq_or_dict.get('z', 0.0))
        else:
            try:
                vx = float(velocity_seq_or_dict[0])
                vy = float(velocity_seq_or_dict[1])
                vz = float(velocity_seq_or_dict[2])
            except Exception:
                vx, vy, vz = 0.0, 0.0, 0.0

        if self.input_frame == 'sumo':
            # Flip Y to match CARLA left-handed axes
            return carla.Vector3D(vx, -vy, vz)
        return carla.Vector3D(vx, vy, vz)

    def _apply_front_bumper_offset_if_needed(self, sumo_loc: List[float], sumo_rot: List[float], maybe_extent_x: Optional[float]) -> Tuple[List[float], List[float]]:
        """
        Mirror BridgeHelper.get_carla_transform bumper-to-center adjustment when inputs are in SUMO frame.
        Expects SUMO location/rotation and an optional extent_x (front bumper distance from center).
        Returns adjusted SUMO location/rotation (still in SUMO frame) ready for _to_carla_* conversion.
        """
        try:
            if maybe_extent_x is None:
                return sumo_loc, sumo_rot
            import math
            pitch = float(sumo_rot[0]) if len(sumo_rot) > 0 else 0.0
            yaw = float(sumo_rot[1]) if len(sumo_rot) > 1 else 0.0
            roll = float(sumo_rot[2]) if len(sumo_rot) > 2 else 0.0
            x = float(sumo_loc[0]); y = float(sumo_loc[1]); z = float(sumo_loc[2] if len(sumo_loc) > 2 else 0.0)
            # BridgeHelper uses yaw' = -yaw + 90 for computing forward axis in SUMO frame
            yaw_prime_deg = -1.0 * yaw + 90.0
            dx = math.cos(math.radians(yaw_prime_deg)) * float(maybe_extent_x)
            dy = math.sin(math.radians(yaw_prime_deg)) * float(maybe_extent_x)
            dz = math.sin(math.radians(pitch)) * float(maybe_extent_x)
            return [x - dx, y - dy, z - dz], [pitch, yaw, roll]
        except Exception:
            return sumo_loc, sumo_rot

    # ----- Coordinate transform configuration -----
    def set_input_frame_mode(self, mode: str) -> bool:
        try:
            mode_l = str(mode).strip().lower()
            if mode_l in ('sumo', 'carla'):
                self.input_frame = 'sumo' if mode_l == 'sumo' else 'carla'
                return True
            return False
        except Exception:
            return False

    def set_net_offset_xy(self, x: float, y: float) -> bool:
        try:
            self.net_offset_xy = (float(x), float(y))
            return True
        except Exception:
            return False

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
                self._odr_to_tl, self._odr_to_tls, self._tl_id_to_odr = build_light_index(self.world)
                    
                logger.info("Connected to CARLA at %s:%s | map=%s",
                            self.carla_host, self.carla_port, self.world.get_map().name)
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
            print("spawn_actor received: type=%s id=%s loc=%s rot=%s attrs=%s", actor_type, actor_id, location, rotation, list((attributes or {}).keys()))
            logger.info("[XMLRPC v0.9] spawn_actor received: type=%s id=%s loc=%s rot=%s attrs=%s", actor_type, actor_id, location, rotation, list((attributes or {}).keys()))
            with self.lock:
                if not self.is_connected(): return False
                if actor_id in self.actors: return False
                bp = self.world.get_blueprint_library().find(actor_type)
                if not bp: return False
                if attributes:
                    for k, v in attributes.items():
                        if bp.has_attribute(k): bp.set_attribute(k, str(v))
                loc = self._to_carla_location(location)
                rot = self._to_carla_rotation(rotation)
                transform = carla.Transform(loc, rot)
                actor = self.world.spawn_actor(bp, transform)
                self.actors[actor_id] = actor
                self.actor_types[actor_id] = actor_type
                self.actor_blueprints[actor_id] = bp
                return True
        except Exception as e:
            print("spawn_actor error: %s", e)
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
                loc = self._to_carla_location(location)
                rot = self._to_carla_rotation(rotation)
                transform = carla.Transform(loc, rot)
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

                # Compatible with multiple input formats and coordinate frames
                if not (hasattr(velocity, '__len__') or isinstance(velocity, dict)):
                    logger.error("update_actor_velocity expects length-3 sequence or dict {x,y,z}")
                    return False
                vec = self._to_carla_velocity(velocity)

                # If target velocity interface exists, use it first (consistent with set_actor_state_properties)
                if hasattr(actor, 'set_target_velocity'):
                    actor.set_target_velocity(vec)
                    return True

                # Otherwise fallback to set_velocity
                if hasattr(actor, 'set_velocity'):
                    try:
                        # If physics simulation can be enabled, try to ensure it is enabled
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
                    loc_dict = t_in.get('location', {})
                    rot_dict = t_in.get('rotation', {})
                    loc_seq = [loc_dict.get('x', 0.0), loc_dict.get('y', 0.0), loc_dict.get('z', 0.0)]
                    rot_seq = [rot_dict.get('pitch', 0.0), rot_dict.get('yaw', 0.0), rot_dict.get('roll', 0.0)]
                    reference = str(t_in.get('reference', 'sumo_center'))
                    extent_x = t_in.get('extent_x', None)
                    if self.input_frame == 'sumo' and reference == 'sumo_front_bumper' and extent_x is not None:
                        loc_seq, rot_seq = self._apply_front_bumper_offset_if_needed(loc_seq, rot_seq, extent_x)
                    loc = self._to_carla_location(loc_seq)
                    rot = self._to_carla_rotation(rot_seq)
                    transform = carla.Transform(loc, rot)
                    actor.set_transform(transform)

                tv = properties_to_set.get('target_velocity')
                if tv:
                    vec = self._to_carla_velocity(tv)
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
                        odr = self._tl_id_to_odr.get(tl.id)
                        if odr is None:
                            continue
                        item = {
                            'opendrive_id': odr,
                            'state': int(self._tl_state_to_int(state)),
                            'elapsed_time': float(getattr(tl, 'get_elapsed_time', lambda: 0.0)()),
                            'timestamp': float(ts)
                        }
                        try: item['is_frozen'] = bool(tl.is_frozen())
                        except Exception: pass
                        try: item['pole_index'] = int(tl.get_pole_index())
                        except Exception: pass
                        out.append(item)
                    except Exception as e:
                        logger.error("Error processing traffic light %s: %s", tl.id, e)
                        continue
                logger.info("Processed get_all_traffic_light_states request.")
                return out
        except Exception as e:
            logger.error("get_all_traffic_light_states error: %s", e)
            return []

    def set_traffic_light_state(self, traffic_light_id: ActorKey, state: str) -> bool:
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                tl = self._odr_to_tl.get(str(traffic_light_id))
                if not tl:
                    print("Traffic light %s not found" % traffic_light_id)
                    return False

                if   state == 'red':
                    tl.set_state(carla.TrafficLightState.Red)
                elif state == 'yellow':
                    tl.set_state(carla.TrafficLightState.Yellow)
                elif state == 'green':
                    tl.set_state(carla.TrafficLightState.Green)
                else:
                    return False
                return True
        except Exception as e:
            logger.error("set_traffic_light_state error: %s", e)
            return False

    def set_traffic_light_timer(self, traffic_light_id: ActorKey, time_s: float) -> bool:
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                tl = self._odr_to_tl.get(str(traffic_light_id))
                if not tl:
                    print("Traffic light %s not found" % traffic_light_id)
                    return False

                state = tl.get_state()
                elapsed = float(tl.get_elapsed_time() or 0.0)
                desired_remaining = max(0.0, float(time_s))
                new_total = elapsed + desired_remaining

                if   state == carla.TrafficLightState.Green:
                    tl.set_green_time(new_total)
                elif state == carla.TrafficLightState.Yellow:
                    tl.set_yellow_time(new_total)
                elif state == carla.TrafficLightState.Red:
                    tl.set_red_time(new_total)
                else:
                    return False

                return True
        except Exception as e:
            logger.error("set_traffic_light_timer error: %s", e)
            return False
        
    def freeze_all_traffic_lights(self, frozen: bool = True) -> int:
        """
        Freeze or unfreeze all traffic lights. Returns count of actors updated.
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return 0
                count = 0
                for tl in self.world.get_actors().filter('traffic.traffic_light'):
                    try:
                        tl.freeze(bool(frozen))
                        count += 1
                    except Exception as e:
                        logger.warning("freeze_all_traffic_lights: failed on %s: %s", tl.id, e)
                logger.info("freeze_all_traffic_lights: set frozen=%s on %d traffic lights", frozen, count)
                return count
        except Exception as e:
            logger.error("freeze_all_traffic_lights error: %s", e)
            return 0


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

                # LiDAR
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

    # ---------- Transform utilities exposed via XML-RPC ----------
    def spawn_actor_from_sumo(self, actor_type: str, actor_id: str,
                               sumo_location: List[float], sumo_rotation: List[float],
                               extent_x: Optional[float] = None,
                               attributes: Dict[str, Any] = None,
                               reference: str = 'sumo_front_bumper') -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                if actor_id in self.actors: return False
                bp = self.world.get_blueprint_library().find(actor_type)
                if not bp: return False
                if attributes:
                    for k, v in attributes.items():
                        if bp.has_attribute(k): bp.set_attribute(k, str(v))
                loc_seq, rot_seq = list(sumo_location), list(sumo_rotation)
                if self.input_frame == 'sumo' and reference == 'sumo_front_bumper' and extent_x is not None:
                    loc_seq, rot_seq = self._apply_front_bumper_offset_if_needed(loc_seq, rot_seq, extent_x)
                transform = carla.Transform(self._to_carla_location(loc_seq), self._to_carla_rotation(rot_seq))
                actor = self.world.spawn_actor(bp, transform)
                self.actors[actor_id] = actor
                self.actor_types[actor_id] = actor_type
                self.actor_blueprints[actor_id] = bp
                return True
        except Exception as e:
            logger.error("spawn_actor_from_sumo error: %s", e)
            return False

    def update_actor_transform_from_sumo(self, actor_key: ActorKey,
                                          sumo_location: List[float], sumo_rotation: List[float],
                                          extent_x: Optional[float] = None,
                                          reference: str = 'sumo_front_bumper') -> bool:
        try:
            with self.lock:
                actor = self._resolve_actor(actor_key)
                if actor is None: return False
                loc_seq, rot_seq = list(sumo_location), list(sumo_rotation)
                if self.input_frame == 'sumo' and reference == 'sumo_front_bumper' and extent_x is not None:
                    loc_seq, rot_seq = self._apply_front_bumper_offset_if_needed(loc_seq, rot_seq, extent_x)
                actor.set_transform(carla.Transform(self._to_carla_location(loc_seq), self._to_carla_rotation(rot_seq)))
                return True
        except Exception as e:
            logger.error("update_actor_transform_from_sumo error: %s", e)
            return False

    def sumo_to_carla_transform(self, sumo_location: List[float], sumo_rotation: List[float],
                                extent_x: Optional[float] = None,
                                reference: str = 'sumo_front_bumper') -> Dict[str, List[float]]:
        try:
            loc_seq, rot_seq = list(sumo_location), list(sumo_rotation)
            if self.input_frame == 'sumo' and reference == 'sumo_front_bumper' and extent_x is not None:
                loc_seq, rot_seq = self._apply_front_bumper_offset_if_needed(loc_seq, rot_seq, extent_x)
            c_loc = self._to_carla_location(loc_seq)
            c_rot = self._to_carla_rotation(rot_seq)
            return {
                'location': [float(c_loc.x), float(c_loc.y), float(c_loc.z)],
                'rotation': [float(c_rot.pitch), float(c_rot.yaw), float(c_rot.roll)]
            }
        except Exception:
            return {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}

    def carla_to_sumo_transform(self, location: List[float], rotation: List[float],
                                extent_x: Optional[float] = None) -> Dict[str, List[float]]:
        """
        Convert CARLA world transform to SUMO frame, mirroring BridgeHelper.get_sumo_transform.
        """
        try:
            import math
            in_x = float(location[0]); in_y = float(location[1]); in_z = float(location[2] if len(location) > 2 else 0.0)
            pitch = float(rotation[0] if len(rotation) > 0 else 0.0)
            yaw = float(rotation[1] if len(rotation) > 1 else 0.0)
            roll = float(rotation[2] if len(rotation) > 2 else 0.0)
            # Transform to SUMO handedness and apply net offset like BridgeHelper.get_sumo_transform
            out_x = in_x + float(self.net_offset_xy[0])
            out_y = -in_y + float(self.net_offset_xy[1])
            out_z = in_z
            # From center to front bumper if extent provided
            if extent_x is not None:
                dx = math.cos(math.radians(-1.0 * yaw)) * float(extent_x)
                dy = -math.sin(math.radians(-1.0 * yaw)) * float(extent_x)
                dz = -math.sin(math.radians(pitch)) * float(extent_x)
                out_x += dx; out_y += dy; out_z += dz
            # SUMO rotation mirrors BridgeHelper's out_rotation (pitch,yaw,roll)
            return {
                'location': [out_x, out_y, out_z],
                'rotation': [pitch, yaw, roll]
            }
        except Exception:
            return {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}

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
            if not self.client: return []
            return list(self.client.get_available_maps())
        except Exception as e:
            logger.error("get_available_maps error: %s", e)
            return []

    def load_map(self, map_name: str) -> bool:
        try:
            with self.lock:
                if not self.is_connected(): return False
                self.world = self.client.load_world(map_name)
                return True
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

def build_light_index(world):
    odr_to_tl = {}
    tl_id_to_odr = {}
    multi = {}
    mp = world.get_map()

    for lm in mp.get_all_landmarks():
        if getattr(lm, "type", None) == getattr(carla, "LandmarkType", None) and \
        lm.type != carla.LandmarkType.TrafficLight:
            continue

        tl = world.get_traffic_light(lm)
        if tl is None:
            continue

        odr = str(lm.id)  # OpenDRIVE signal id
        odr_to_tl.setdefault(odr, tl)
        tl_id_to_odr[tl.id] = odr
        multi.setdefault(odr, []).append(tl)

    return odr_to_tl, multi, tl_id_to_odr

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
