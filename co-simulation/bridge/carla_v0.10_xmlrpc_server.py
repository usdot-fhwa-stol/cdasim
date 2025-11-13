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
import math
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
                 carla_host: str = 'localhost', carla_port: int = 2000,
                 tls_manager: str = 'none', phase: float = 0.1, default_map_name: str = 'Town04'):
        self.host = host
        self.port = port
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.tls_manager = tls_manager
        self.phase = phase
        self.default_map_name = default_map_name

        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None

        self.actors: Dict[str, carla.Actor] = {}
        self.actor_types: Dict[str, str] = {}
        self.actor_blueprints: Dict[str, carla.ActorBlueprint] = {}

        self.sensors: Dict[str, carla.Sensor] = {}
        self.sensor_data: Dict[str, Any] = {}
        self.sensor_blueprints: Dict[str, carla.ActorBlueprint] = {}

        self.server = SimpleXMLRPCServer(
            (host, port),
            requestHandler=SimpleXMLRPCRequestHandler,
            allow_none=True, logRequests=False
        )
        self._register_methods()
        logger.info("XML-RPC methods registered")

    def _apply_sync_settings(self) -> bool:
        """
        Ensure CARLA runs in synchronous mode with the configured fixed delta.
        Must be called after any map/world reload as settings reset on load.
        """
        try:
            if self.world is None:
                return False
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.phase
            self.world.apply_settings(settings)
            logger.debug("Applied synchronous settings (fixed_delta_seconds=%.3f)", self.phase)
            return True
        except Exception as e:
            logger.exception("Failed to apply synchronous settings: %s", e)
            return False

    def _safe_try_spawn(self, bp: carla.ActorBlueprint, base_transform: carla.Transform) -> Optional[carla.Actor]:
        """
        Try to spawn an actor safely by attempting multiple height offsets and small XY jitters
        using world.try_spawn_actor. Returns the actor on success or None if all attempts fail.
        """
        if not self.world:
            return None
        # Heights (meters) to try to avoid ground collisions; small to large
        height_offsets = [0.0, 0.2, 0.5, 1.0]
        # Small xy jitters (meters)
        xy_jitters = [(0.0, 0.0), (0.2, 0.0), (-0.2, 0.0), (0.0, 0.2), (0.0, -0.2), (0.2, 0.2), (-0.2, 0.2), (0.2, -0.2), (-0.2, -0.2)]

        for dz in height_offsets:
            for dx, dy in xy_jitters:
                try:
                    t = carla.Transform(
                        carla.Location(base_transform.location.x + dx,
                                       base_transform.location.y + dy,
                                       base_transform.location.z + dz),
                        base_transform.rotation
                    )
                    actor = self.world.try_spawn_actor(bp, t)
                    if actor is not None:
                        return actor
                except Exception:
                    continue
        return None

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
        self.server.register_function(self.get_detected_objects, 'get_detected_objects')

        # Spectator / camera utilities
        self.server.register_function(self.set_spectator_to_actor, 'set_spectator_to_actor')

        # Maps
        self.server.register_function(self.get_map_name, 'get_map_name')
        self.server.register_function(self.get_available_maps, 'get_available_maps')
        self.server.register_function(self.load_map, 'load_map')

        # Coordinate transform configuration
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
        except Exception as e:
            logger.exception("Error getting simulation timestamp: %s", e)
            return 0.0



    # ----- Coordinate transform configuration -----

    def set_net_offset_xy(self, x: float, y: float) -> bool:
        try:
            self.net_offset_xy = (float(x), float(y))
            return True
        except Exception as e:
            logger.exception("Error setting net offset (x=%s, y=%s): %s", x, y, e)
            return False

    def _resolve_actor(self, actor_key: ActorKey) -> Optional[carla.Actor]:
        if isinstance(actor_key, str):
            if actor_key in self.actors:
                return self.actors[actor_key]
            try:
                aid = int(actor_key)
                if self.world:
                    return self.world.get_actor(aid)
            except Exception as e:
                logger.exception("Error resolving actor by ID (actor_key=%s): %s", actor_key, e)
                return None
        else:
            try:
                if self.world:
                    return self.world.get_actor(int(actor_key))
            except Exception as e:
                logger.exception("Error resolving actor by int key (actor_key=%s): %s", actor_key, e)
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
            except Exception as e:
                logger.exception("Error resolving sensor by ID (sensor_key=%s): %s", sensor_key, e)
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
            if self.client is None:
                self.client = carla.Client(self.carla_host, self.carla_port)
                self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            # Set CARLA simulation to passive mode (synchronous mode)
            self._apply_sync_settings()
            logger.info("CARLA simulation mode set to passive (synchronous) with phase=%.3f", self.phase)
            
            # Get current map name
            try:
                current_map = self.world.get_map().name
            except Exception as e:
                logger.exception("Error getting current map name: %s", e)
                current_map = "<unknown>"
            logger.info("Connected to CARLA %s at %s:%s | current map=%s",
                        CARLA_VERSION, self.carla_host, self.carla_port, current_map)
            
            # Automatically load default map if not already loaded
            if current_map != self.default_map_name:
                logger.info("Current map is not %s, attempting to load %s...", self.default_map_name, self.default_map_name)
                try:
                    self.world = self.client.load_world(self.default_map_name)
                    # Re-apply synchronous settings after world reload
                    self._apply_sync_settings()
                    new_map = self.world.get_map().name
                    logger.info("Successfully loaded %s map: %s", self.default_map_name, new_map)
                except Exception as load_error:
                    logger.error("Failed to load %s map: %s", self.default_map_name, load_error)
                    # Continue with current map if default map loading fails
                    logger.warning("Continuing with current map: %s", current_map)
            else:
                logger.info("%s map is already loaded", self.default_map_name)
            
            # Configure TLS manager after successful connection
            self._configure_tls_manager()
            
            return True
        except Exception as e:
            logger.error("Failed to connect: %s", e)
            return False

    def disconnect(self) -> bool:
        try:
            
            for _, a in list(self.actors.items()):
                try: a.destroy()
                except Exception as e: 
                    logger.exception("Error destroying actor (actor_id=%s): %s", getattr(a, 'id', 'unknown'), e)
            for _, s in list(self.sensors.items()):
                try: s.destroy()
                except Exception as e:
                    logger.exception("Error destroying sensor (sensor_id=%s): %s", getattr(s, 'id', 'unknown'), e)
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

    def _configure_tls_manager(self):
        """
        Configure traffic light manager based on tls_manager parameter.
        This method should be called after connecting to CARLA.
        """
        try:
            if not self.is_connected():
                return False
            
            if self.tls_manager == 'carla':
                # CARLA manages traffic lights - disable SUMO traffic light control
                logger.info("TLS Manager: CARLA will manage traffic lights")
                # Note: In a bridge context, this would disable SUMO traffic light control
                # For XML-RPC server, we just log the configuration
                
            elif self.tls_manager == 'sumo':
                # SUMO manages traffic lights - disable CARLA traffic light control
                logger.info("TLS Manager: SUMO will manage traffic lights")
                # Note: In a bridge context, this would disable CARLA traffic light control
                # For XML-RPC server, we just log the configuration
                
            elif self.tls_manager == 'EVC':
                # EVC manages traffic lights
                logger.info("TLS Manager: EVC will manage traffic lights")
                # Note: In a bridge context, this would disable CARLA traffic light control
                # For XML-RPC server, we just log the configuration
                
            else:  # 'none' or any other value
                logger.info("TLS Manager: No traffic light management")
                
            return True
        except Exception as e:
            logger.error("Error configuring TLS manager: %s", e)
            return False

    # ---------- Simulation ----------
    def advance_simulation(self) -> bool:
        try:
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
                    attributes: Dict[str, Any] = None) -> Union[bool, str]:
        try:
            
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
                except Exception as e:
                    logger.debug("Blueprint find failed for %s: %s", req, e)
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

            # Get vehicle extent for proper center calculation
            extent_x = 0.0
            if bp.has_attribute('extent_x'):
                try:
                    extent_x = float(bp.get_attribute('extent_x').as_str())
                except Exception as e:
                    logger.debug("Error getting extent_x attribute: %s", e)
                    pass

            try:
                lx = float(location[0]); ly = float(location[1]); lz = float(location[2]) if len(location) > 2 else 0.0
            except Exception as e:
                logger.exception("Error converting location in spawn_actor (location=%s): %s", location, e)
                lx, ly, lz = 0.0, 0.0, 0.0
            try:
                    rp = float(rotation[0]) if len(rotation) > 0 else 0.0
                    ry = float(rotation[1]) if len(rotation) > 1 else 0.0
                    rr = float(rotation[2]) if len(rotation) > 2 else 0.0
            except Exception as e:
                    logger.exception("Error converting rotation in spawn_actor (rotation=%s): %s", rotation, e)
                    rp, ry, rr = 0.0, 0.0, 0.0
            loc = carla.Location(lx, ly, lz)
            rot = carla.Rotation(rp, ry, rr)
            transform = carla.Transform(loc, rot)
            
            logger.info(
                "========spawn_actor received: actor %s of type %s "
                "loc=(%.3f, %.3f, %.3f) "
                "rot=(pitch=%.1f, yaw=%.1f, roll=%.1f) "
                "attributes=%s========",
                actor_id, actor_type, loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll, attributes
            )
            logger.info("spawn actor at loc=%.3f, %.3f, %.3f, rot=%.1f, %.1f, %.1f", loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll)
            # Attempt safe spawn with collision avoidance (height offsets and slight jitters)
            actor = self._safe_try_spawn(bp, transform)
            if actor is None:
                logger.warning("spawn_actor blocked or invalid at loc=(%.2f, %.2f, %.2f)", loc.x, loc.y, loc.z)
                return False
            self.actors[actor_id] = actor
            self.actor_types[actor_id] = actor_type
            self.actor_blueprints[actor_id] = bp
            
            # Only switch spectator to the first spawned actor
            if len(self.actors) == 1:  # Only for the first vehicle
                try:
                    self._set_spectator_to_actor_object(actor, 'follow', 8.0, 3.0, -20.0)
                    logger.info("========Spectator switched to follow first actor: %s========", actor_id)
                except Exception as e:
                    logger.error("Failed to switch spectator to first actor %s: %s", actor_id, e)
            else:
                logger.info("========Actor %s spawned (spectator not moved)========", actor_id)
            
            logger.info("========spawn_actor success========")
            # Return the CARLA internal actor ID instead of boolean
            return str(actor.id)
        except Exception as e:
            logger.error("spawn_actor error: %s", e)
            return False


    def destroy_actor(self, actor_key: ActorKey) -> bool:
        try:
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
            actor = self._resolve_actor(actor_key)
            if actor is None: return False
            
            # Get vehicle extent for proper center calculation
            extent_x = 0.0
            if hasattr(actor, 'bounding_box') and hasattr(actor.bounding_box, 'extent'):
                extent_x = float(actor.bounding_box.extent.x)
            
            try:
                lx = float(location[0]); ly = float(location[1]); lz = float(location[2]) if len(location) > 2 else 0.0
            except Exception as e:
                logger.exception("Error converting location in update_actor_transform (location=%s): %s", location, e)
                lx, ly, lz = 0.0, 0.0, 0.0
            try:
                rp = float(rotation[0]) if len(rotation) > 0 else 0.0
                ry = float(rotation[1]) if len(rotation) > 1 else 0.0
                rr = float(rotation[2]) if len(rotation) > 2 else 0.0
            except Exception as e:
                logger.exception("Error converting rotation in update_actor_transform (rotation=%s): %s", rotation, e)
                rp, ry, rr = 0.0, 0.0, 0.0
            transform = carla.Transform(carla.Location(lx, ly, lz), carla.Rotation(rp, ry, rr))
            actor.set_transform(transform)
            return True
        except Exception as e:
            logger.error("update_actor_transform error: %s", e)
            return False

    def update_actor_velocity(self, actor_key: ActorKey, velocity: List[float]) -> bool:
        try:
            
            actor = self._resolve_actor(actor_key)
            if actor is None:
                return False

            if not (hasattr(velocity, '__len__') or isinstance(velocity, dict)):
                logger.error("update_actor_velocity expects length-3 sequence or dict {x,y,z}")
                return False
            try:
                vx = float(velocity[0]); vy = float(velocity[1]); vz = float(velocity[2])
            except Exception as e:
                logger.exception("Error converting velocity list in update_actor_velocity (velocity=%s): %s", velocity, e)
                vx, vy, vz = 0.0, 0.0, 0.0
            vec = carla.Vector3D(vx, vy, vz)

            if hasattr(actor, 'set_target_velocity'):
                actor.set_target_velocity(vec)
                return True

            if hasattr(actor, 'set_velocity'):
                try:
                    if hasattr(actor, 'set_simulate_physics'):
                        try:
                            actor.set_simulate_physics(True)
                        except Exception as e:
                            logger.debug("Error setting simulate_physics: %s", e)
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
            
            if not self.is_connected():
                return {}
            
            out = {}
            # Get actual actors from CARLA world to detect externally removed actors
            world_actors = {}
            try:
                for actor in self.world.get_actors():
                    world_actors[str(actor.id)] = actor
            except Exception as e:
                logger.warning("Failed to get world actors: %s", e)
                world_actors = {}
            
            # Clean up actors that no longer exist in CARLA world
            actors_to_remove = []
            for alias, actor in self.actors.items():
                if str(actor.id) not in world_actors:
                    actors_to_remove.append(alias)
                    logger.info("Actor %s (ID: %s) no longer exists in CARLA world, removing from tracking", alias, actor.id)
            
            for alias in actors_to_remove:
                self.actors.pop(alias, None)
                self.actor_types.pop(alias, None)
                self.actor_blueprints.pop(alias, None)
            
            # Build output with currently existing actors
            for alias, actor in self.actors.items():
                try:
                    # Double-check actor still exists and is valid
                    if str(actor.id) in world_actors and hasattr(actor, 'get_transform'):
                        t = actor.get_transform()
                        actor_data = {
                            'type': self.actor_types.get(alias, getattr(actor, 'type_id', '')),
                            'transform': {
                                'location': [float(t.location.x), float(t.location.y), float(t.location.z)],
                                'rotation': [float(t.rotation.pitch), float(t.rotation.yaw), float(t.rotation.roll)]
                            }
                        }
                        
                        # Add velocity information if available
                        if hasattr(actor, 'get_velocity'):
                            try:
                                v = actor.get_velocity()
                                actor_data['velocity'] = {
                                    'linear': [float(v.x), float(v.y), float(v.z)]
                                }
                            except Exception as e:
                                logger.debug("Failed to get velocity for actor %s: %s", alias, e)
                                actor_data['velocity'] = {'linear': [0.0, 0.0, 0.0]}
                        else:
                            actor_data['velocity'] = {'linear': [0.0, 0.0, 0.0]}
                        
                        out[alias] = actor_data
                    else:
                        logger.warning("Actor %s (ID: %s) is invalid, skipping", alias, actor.id)
                except Exception as e:
                    logger.warning("Failed to get transform for actor %s (ID: %s): %s", alias, actor.id, e)
                    # Remove invalid actor from tracking
                    self.actors.pop(alias, None)
                    self.actor_types.pop(alias, None)
                    self.actor_blueprints.pop(alias, None)
                return out
        except Exception as e:
            logger.error("get_all_actors error: %s", e)
            return {}

    # ---------- Actor Data (spec) ----------
    def get_active_actor_ids(self, filter_pattern: str = "vehicle.*") -> List[int]:
        try:
            
                if not self.is_connected(): return []
                return [int(a.id) for a in self.world.get_actors().filter(filter_pattern)]
        except Exception as e:
            logger.error("get_active_actor_ids error: %s", e)
            return []

    def get_actor_basic_info(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            
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
            
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_velocity'): return None
                v = actor.get_velocity()
                return {'x': float(v.x), 'y': float(v.y), 'z': float(v.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_velocity error: %s", e)
            return None

    def get_actor_acceleration(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_acceleration'): return None
                a = actor.get_acceleration()
                return {'x': float(a.x), 'y': float(a.y), 'z': float(a.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_acceleration error: %s", e)
            return None

    def get_actor_angular_velocity(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            
                actor = self._resolve_actor(actor_key)
                if actor is None or not hasattr(actor, 'get_angular_velocity'): return None
                w = actor.get_angular_velocity()
                return {'x': float(w.x), 'y': float(w.y), 'z': float(w.z), 'timestamp': self._sim_timestamp()}
        except Exception as e:
            logger.error("get_actor_angular_velocity error: %s", e)
            return None

    def get_actor_bounding_box(self, actor_key: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            
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
        actor = self._resolve_actor(actor_key)
        if actor is None: return False
        t_in = properties_to_set.get('transform')
        if t_in:
            loc_dict = t_in.get('location', {})
            rot_dict = t_in.get('rotation', {})
            loc_seq = [loc_dict.get('x', 0.0), loc_dict.get('y', 0.0), loc_dict.get('z', 0.0)]
            rot_seq = [rot_dict.get('pitch', 0.0), rot_dict.get('yaw', 0.0), rot_dict.get('roll', 0.0)]
            lx = float(loc_seq[0]); ly = float(loc_seq[1]); lz = float(loc_seq[2]) if len(loc_seq) > 2 else 0.0
            rp = float(rot_seq[0]) if len(rot_seq) > 0 else 0.0
            ry = float(rot_seq[1]) if len(rot_seq) > 1 else 0.0
            rr = float(rot_seq[2]) if len(rot_seq) > 2 else 0.0
            loc = carla.Location(lx, ly, lz)
            rot = carla.Rotation(rp, ry, rr)
            transform = carla.Transform(loc, rot)
            actor.set_transform(transform)

            tv = properties_to_set.get('target_velocity')
            if tv:
                if isinstance(tv, dict):
                    vx = float(tv.get('x', 0.0)); vy = float(tv.get('y', 0.0)); vz = float(tv.get('z', 0.0))
                else:
                    vx = float(tv[0]); vy = float(tv[1]); vz = float(tv[2])
                    vec = carla.Vector3D(vx, vy, vz)
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

    # ---------- Traffic Lights ----------
    def _tl_state_to_int(self, state: carla.TrafficLightState) -> int:
        if state == carla.TrafficLightState.Red: return 0
        if state == carla.TrafficLightState.Yellow: return 1
        if state == carla.TrafficLightState.Green: return 2
        if state == carla.TrafficLightState.Off: return 3
        return 4

    def get_traffic_light_state(self, traffic_light_id: ActorKey) -> Optional[Dict[str, Any]]:
        try:
            
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
                except Exception as e: 
                    logger.debug("Error getting is_frozen for traffic light %s: %s", tid, e)
                try: data['pole_index'] = int(tl.get_pole_index())
                except Exception as e:
                    logger.debug("Error getting pole_index for traffic light %s: %s", tid, e)
                return data
        except Exception as e:
            logger.error("get_traffic_light_state error: %s", e)
            return None

    def get_all_traffic_light_states(self) -> List[Dict[str, Any]]:
        try:
            
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
                        except Exception as e: 
                            logger.debug("Error getting is_frozen for traffic light %s: %s", tl.id, e)
                        try: item['pole_index'] = int(tl.get_pole_index())
                        except Exception as e:
                            logger.debug("Error getting pole_index for traffic light %s: %s", tl.id, e)
                        out.append(item)
                    except Exception as e:
                        logger.exception("Error processing traffic light %s: %s", getattr(tl, 'id', 'unknown'), e)
                        continue
                return out
        except Exception as e:
            logger.error("get_all_traffic_light_states error: %s", e)
            return []

    def set_traffic_light_state(self, traffic_light_id: ActorKey, state: str) -> bool:
        try:
            
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
    def get_detected_objects(self, infrastructure_id: str, sensor_key: SensorKey) -> str:
        # Placeholder: return empty JSON array
        return json.dumps([])

    # ---------- Spectator Utilities ----------
    def _set_spectator_to_actor_object(self, actor: carla.Actor, preset: str = 'follow', back: float = 10.0, up: float = 5.0, pitch: float = -15.0) -> bool:
        """
        Set spectator to follow a specific actor object directly (internal use).
        This avoids the need to resolve actor by ID and is more reliable.
        """
        try:
            
                if not self.is_connected():
                    return False
                if actor is None:
                    return False
                t = actor.get_transform()
                spectator = self.world.get_spectator()
                if spectator is None:
                    return False

                # Ensure minimum values to avoid camera being too close to vehicle
                back = max(3.0, float(back))  # Minimum 3 meters back
                up = max(1.0, float(up))      # Minimum 1 meter up
                pitch = float(pitch)

                preset_l = str(preset).strip().lower()
                if preset_l == 'topdown':
                    cam_loc = carla.Location(t.location.x, t.location.y, t.location.z + abs(up))
                    cam_rot = carla.Rotation(pitch=-90.0, yaw=t.rotation.yaw, roll=0.0)
                else:  # 'follow' default
                    try:
                        # Calculate camera position behind the vehicle
                        # CARLA uses right-handed coordinate system: +X forward, +Y right, +Z up
                        yaw_rad = math.radians(t.rotation.yaw)
                        # Position camera behind the vehicle (opposite to vehicle's forward direction)
                        dx = -back * math.cos(yaw_rad)  # Negative because we want to be behind
                        dy = -back * math.sin(yaw_rad)  # Negative because we want to be behind
                    except Exception as e:
                        logger.exception("Error calculating camera position for spectator (yaw=%s, back=%s): %s", t.rotation.yaw, back, e)
                        dx, dy = -back, 0.0
                    cam_loc = carla.Location(t.location.x + dx, t.location.y + dy, t.location.z + up)
                    cam_rot = carla.Rotation(pitch=pitch, yaw=t.rotation.yaw, roll=0.0)

                spectator.set_transform(carla.Transform(cam_loc, cam_rot))
                logger.info("Spectator positioned to actor object: back=%.1f, up=%.1f, pitch=%.1f", back, up, pitch)
                return True
        except Exception as e:
            logger.error("_set_spectator_to_actor_object error: %s", e)
            return False

    def set_spectator_to_actor(self, actor_key: ActorKey, preset: str = 'follow', back: float = 10.0, up: float = 5.0, pitch: float = -15.0) -> bool:
        try:
            
                if not self.is_connected():
                    return False
                actor = self._resolve_actor(actor_key)
                if actor is None:
                    return False
                t = actor.get_transform()
                spectator = self.world.get_spectator()
                if spectator is None:
                    return False

                preset_l = str(preset).strip().lower()
                if preset_l == 'topdown':
                    cam_loc = carla.Location(t.location.x, t.location.y, t.location.z + abs(up))
                    cam_rot = carla.Rotation(pitch=-90.0, yaw=t.rotation.yaw, roll=0.0)
                else:  # 'follow' default
                    try:
                        yaw_rad = math.radians(t.rotation.yaw)
                        dx = float(back) * math.cos(yaw_rad)
                        dy = float(back) * math.sin(yaw_rad)
                    except Exception as e:
                        logger.exception("Error calculating camera position for set_spectator_to_actor (yaw=%s, back=%s): %s", t.rotation.yaw, back, e)
                        dx, dy = float(back), 0.0
                    cam_loc = carla.Location(t.location.x - dx, t.location.y - dy, t.location.z + float(up))
                    cam_rot = carla.Rotation(pitch=float(pitch), yaw=t.rotation.yaw, roll=0.0)

                spectator.set_transform(carla.Transform(cam_loc, cam_rot))
                return True
        except Exception as e:
            logger.error("set_spectator_to_actor error: %s", e)
            return False

    # ---------- Maps ----------
    def get_map_name(self) -> str:
        try:
            
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
                # v0.10 may still provide this method, but resources are limited (mostly Town10)
                maps = list(self.client.get_available_maps())
            except Exception as e:
                logger.debug("Error getting available maps: %s", e)
                maps = []
            # v0.10 officially only guarantees Town10 upgrade; provide fallback if query is empty
            if not maps:
                maps = ['Carla/Maps/Town10HD_Opt', 'Carla/Maps/Town10HD']
            return maps
        except Exception as e:
            logger.error("get_available_maps error: %s", e)
            return []

    def load_map(self, map_name: str) -> bool:
        try:
            
                if not self.is_connected():
                    logger.error("load_map failed: Not connected to CARLA")
                    return False
                
                logger.info("Attempting to load map: %s", map_name)
                
                # Try to load the map directly
                try:
                    self.world = self.client.load_world(map_name)
                    # Re-apply synchronous settings after world reload
                    self._apply_sync_settings()
                    
                    # Verify the map was loaded successfully
                    try:
                        new_map = self.world.get_map().name
                        logger.info("Successfully loaded map: %s", new_map)
                    except Exception as e:
                        logger.exception("Could not verify loaded map: %s", e)
                        return False
                    
                    return True
                except Exception as e:
                    # v0.10 lacks many old maps; return False instead of throwing exception when unavailable
                    logger.warning("load_map failed for %s on v0.10: %s", map_name, e)
                    return False
        except Exception as e:
            logger.error("load_map error for map '%s': %s", map_name, e)
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
    parser.add_argument('--phase', type=float, default=0.1, help='Fixed delta seconds for simulation (default: 0.1)')
    parser.add_argument('--map', '--map-name', dest='map_name', default='Town04', help='Default map name to load on connection (default: Town04)')
    parser.add_argument('--tls-manager',
                       type=str,
                       choices=['none', 'sumo', 'carla', 'EVC'],
                       help="select traffic light manager (default: none)",
                       default='none')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    server = CarlaXMLRPCServer(args.host, args.port, args.carla_host, args.carla_port, args.tls_manager, args.phase, args.map_name)
    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Interrupted")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
