#!/usr/bin/env python3
"""
CARLA XML-RPC Server for MOSAIC Integration

This module implements an XML-RPC server that provides a dedicated interface
for MOSAIC to control CARLA actors (vehicles, pedestrians) and traffic lights.
It replaces the TraCI-based bridge with a more suitable XML-RPC architecture.

Copyright (c) 2024 CARLA-MOSAIC Integration Team
"""

import argparse
import logging
import sys
import os
import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple, Any
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

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
import glob

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CarlaXMLRPCServer:
    """
    XML-RPC Server for CARLA-MOSAIC integration.
    
    Provides endpoints for:
    - Actor management (spawn, update, destroy)
    - Traffic light control
    - Simulation control
    - Sensor management
    """
    
    def __init__(self, host: str = 'localhost', port: int = 8090, 
                 carla_host: str = 'localhost', carla_port: int = 2000):
        """
        Initialize the CARLA XML-RPC server.
        
        Args:
            host: Host address for the XML-RPC server
            port: Port for the XML-RPC server
            carla_host: CARLA server host
            carla_port: CARLA server port
        """
        self.host = host
        self.port = port
        self.carla_host = carla_host
        self.carla_port = carla_port
        
        # CARLA client and world
        self.client = None
        self.world = None
        
        # Actor tracking
        self.actors: Dict[str, carla.Actor] = {}
        self.actor_types: Dict[str, str] = {}
        self.actor_blueprints: Dict[str, carla.ActorBlueprint] = {}
        
        # Traffic light tracking
        self.traffic_lights: Dict[str, carla.TrafficLight] = {}
        
        # Sensor tracking
        self.sensors: Dict[str, carla.Sensor] = {}
        self.sensor_data: Dict[str, Any] = {}
        
        # Simulation state
        self.simulation_running = False
        self.simulation_time = 0.0
        
        # Threading
        self.lock = threading.RLock()
        
        # Initialize XML-RPC server
        self.server = SimpleXMLRPCServer((host, port), 
                                        requestHandler=SimpleXMLRPCRequestHandler,
                                        allow_none=True)
        self._register_methods()
        
    def _register_methods(self):
        """Register all XML-RPC methods."""
        # Connection methods
        self.server.register_function(self.connect, 'connect')
        self.server.register_function(self.disconnect, 'disconnect')
        self.server.register_function(self.is_connected, 'is_connected')
        
        # Simulation control
        self.server.register_function(self.start_simulation, 'start_simulation')
        self.server.register_function(self.stop_simulation, 'stop_simulation')
        self.server.register_function(self.step_simulation, 'step_simulation')
        self.server.register_function(self.get_simulation_time, 'get_simulation_time')
        
        # Actor management
        self.server.register_function(self.spawn_actor, 'spawn_actor')
        self.server.register_function(self.destroy_actor, 'destroy_actor')
        self.server.register_function(self.update_actor_transform, 'update_actor_transform')
        self.server.register_function(self.update_actor_velocity, 'update_actor_velocity')
        self.server.register_function(self.get_actor_transform, 'get_actor_transform')
        self.server.register_function(self.get_actor_velocity, 'get_actor_velocity')
        self.server.register_function(self.get_all_actors, 'get_all_actors')
        
        # Traffic light management
        self.server.register_function(self.get_traffic_lights, 'get_traffic_lights')
        self.server.register_function(self.set_traffic_light_state, 'set_traffic_light_state')
        self.server.register_function(self.get_traffic_light_state, 'get_traffic_light_state')
        self.server.register_function(self.set_traffic_light_timer, 'set_traffic_light_timer')
        
        # Sensor management
        self.server.register_function(self.create_sensor, 'create_sensor')
        self.server.register_function(self.destroy_sensor, 'destroy_sensor')
        self.server.register_function(self.get_sensor_data, 'get_sensor_data')
        
        # Map and world information
        self.server.register_function(self.get_map_name, 'get_map_name')
        self.server.register_function(self.get_available_maps, 'get_available_maps')
        self.server.register_function(self.load_map, 'load_map')
        
        logger.info("XML-RPC methods registered successfully")
    
    def connect(self) -> bool:
        """
        Connect to CARLA server.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            with self.lock:
                if self.client is None:
                    self.client = carla.Client(self.carla_host, self.carla_port)
                    self.client.set_timeout(10.0)
                
                self.world = self.client.get_world()
                logger.info(f"Connected to CARLA server at {self.carla_host}:{self.carla_port}")
                logger.info(f"Current map: {self.world.get_map().name}")
                return True
                
        except Exception as e:
            logger.error(f"Failed to connect to CARLA: {e}")
            return False
    
    def disconnect(self) -> bool:
        """
        Disconnect from CARLA server.
        
        Returns:
            True if disconnection successful, False otherwise
        """
        try:
            with self.lock:
                # Destroy all actors
                for actor_id, actor in self.actors.items():
                    try:
                        actor.destroy()
                    except:
                        pass
                
                self.actors.clear()
                self.actor_types.clear()
                self.actor_blueprints.clear()
                self.traffic_lights.clear()
                self.sensors.clear()
                self.sensor_data.clear()
                
                if self.client:
                    self.client = None
                    self.world = None
                
                logger.info("Disconnected from CARLA server")
                return True
                
        except Exception as e:
            logger.error(f"Error during disconnect: {e}")
            return False
    
    def is_connected(self) -> bool:
        """
        Check if connected to CARLA server.
        
        Returns:
            True if connected, False otherwise
        """
        return self.client is not None and self.world is not None
    
    def start_simulation(self) -> bool:
        """
        Start the simulation.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                self.simulation_running = True
                self.simulation_time = 0.0
                logger.info("Simulation started")
                return True
                
        except Exception as e:
            logger.error(f"Error starting simulation: {e}")
            return False
    
    def stop_simulation(self) -> bool:
        """
        Stop the simulation.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                self.simulation_running = False
                logger.info("Simulation stopped")
                return True
                
        except Exception as e:
            logger.error(f"Error stopping simulation: {e}")
            return False
    
    def step_simulation(self, delta_time: float) -> bool:
        """
        Step the simulation by the given time.
        
        Args:
            delta_time: Time step in seconds
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected() or not self.simulation_running:
                    return False
                
                # Update simulation time
                self.simulation_time += delta_time
                
                # Update sensor data
                self._update_sensors()
                
                return True
                
        except Exception as e:
            logger.error(f"Error stepping simulation: {e}")
            return False
    
    def get_simulation_time(self) -> float:
        """
        Get current simulation time.
        
        Returns:
            Current simulation time in seconds
        """
        return self.simulation_time
    
    def spawn_actor(self, actor_type: str, actor_id: str, 
                   location: List[float], rotation: List[float],
                   attributes: Dict[str, Any] = None) -> bool:
        """
        Spawn an actor in CARLA.
        
        Args:
            actor_type: Type of actor (e.g., 'vehicle.tesla.model3', 'walker.pedestrian.0001')
            actor_id: Unique identifier for the actor
            location: [x, y, z] coordinates
            rotation: [pitch, yaw, roll] in degrees
            attributes: Additional attributes for the actor
            
        Returns:
            True if spawn successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                if actor_id in self.actors:
                    logger.warning(f"Actor {actor_id} already exists")
                    return False
                
                # Get blueprint
                blueprint = self.world.get_blueprint_library().find(actor_type)
                if not blueprint:
                    logger.error(f"Blueprint {actor_type} not found")
                    return False
                
                # Set attributes
                if attributes:
                    for key, value in attributes.items():
                        if blueprint.has_attribute(key):
                            blueprint.set_attribute(key, str(value))
                
                # Create transform
                transform = carla.Transform(
                    carla.Location(location[0], location[1], location[2]),
                    carla.Rotation(rotation[0], rotation[1], rotation[2])
                )
                
                # Spawn actor
                actor = self.world.spawn_actor(blueprint, transform)
                
                # Track actor
                self.actors[actor_id] = actor
                self.actor_types[actor_id] = actor_type
                self.actor_blueprints[actor_id] = blueprint
                
                logger.info(f"Spawned actor {actor_id} of type {actor_type}")
                return True
                
        except Exception as e:
            logger.error(f"Error spawning actor {actor_id}: {e}")
            return False
    
    def destroy_actor(self, actor_id: str) -> bool:
        """
        Destroy an actor.
        
        Args:
            actor_id: Unique identifier for the actor
            
        Returns:
            True if destroy successful, False otherwise
        """
        try:
            with self.lock:
                if actor_id not in self.actors:
                    logger.warning(f"Actor {actor_id} not found")
                    return False
                
                actor = self.actors[actor_id]
                actor.destroy()
                
                # Remove from tracking
                del self.actors[actor_id]
                del self.actor_types[actor_id]
                del self.actor_blueprints[actor_id]
                
                logger.info(f"Destroyed actor {actor_id}")
                return True
                
        except Exception as e:
            logger.error(f"Error destroying actor {actor_id}: {e}")
            return False
    
    def update_actor_transform(self, actor_id: str, 
                             location: List[float], rotation: List[float]) -> bool:
        """
        Update actor transform.
        
        Args:
            actor_id: Unique identifier for the actor
            location: [x, y, z] coordinates
            rotation: [pitch, yaw, roll] in degrees
            
        Returns:
            True if update successful, False otherwise
        """
        try:
            with self.lock:
                if actor_id not in self.actors:
                    return False
                
                actor = self.actors[actor_id]
                transform = carla.Transform(
                    carla.Location(location[0], location[1], location[2]),
                    carla.Rotation(rotation[0], rotation[1], rotation[2])
                )
                actor.set_transform(transform)
                return True
                
        except Exception as e:
            logger.error(f"Error updating transform for actor {actor_id}: {e}")
            return False
    
    def update_actor_velocity(self, actor_id: str, velocity: List[float]) -> bool:
        """
        Update actor velocity.
        
        Args:
            actor_id: Unique identifier for the actor
            velocity: [x, y, z] velocity components
            
        Returns:
            True if update successful, False otherwise
        """
        try:
            with self.lock:
                if actor_id not in self.actors:
                    return False
                
                actor = self.actors[actor_id]
                if hasattr(actor, 'set_velocity'):
                    actor.set_velocity(carla.Vector3D(velocity[0], velocity[1], velocity[2]))
                    return True
                return False
                
        except Exception as e:
            logger.error(f"Error updating velocity for actor {actor_id}: {e}")
            return False
    
    def get_actor_transform(self, actor_id: str) -> Optional[Dict[str, List[float]]]:
        """
        Get actor transform.
        
        Args:
            actor_id: Unique identifier for the actor
            
        Returns:
            Dictionary with 'location' and 'rotation' lists, or None if not found
        """
        try:
            with self.lock:
                if actor_id not in self.actors:
                    return None
                
                actor = self.actors[actor_id]
                transform = actor.get_transform()
                
                return {
                    'location': [transform.location.x, transform.location.y, transform.location.z],
                    'rotation': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
                }
                
        except Exception as e:
            logger.error(f"Error getting transform for actor {actor_id}: {e}")
            return None
    
    def get_actor_velocity(self, actor_id: str) -> Optional[List[float]]:
        """
        Get actor velocity.
        
        Args:
            actor_id: Unique identifier for the actor
            
        Returns:
            [x, y, z] velocity components, or None if not found
        """
        try:
            with self.lock:
                if actor_id not in self.actors:
                    return None
                
                actor = self.actors[actor_id]
                if hasattr(actor, 'get_velocity'):
                    velocity = actor.get_velocity()
                    return [velocity.x, velocity.y, velocity.z]
                return None
                
        except Exception as e:
            logger.error(f"Error getting velocity for actor {actor_id}: {e}")
            return None
    
    def get_all_actors(self) -> Dict[str, Dict[str, Any]]:
        """
        Get information about all actors.
        
        Returns:
            Dictionary mapping actor IDs to actor information
        """
        try:
            with self.lock:
                result = {}
                for actor_id, actor in self.actors.items():
                    transform = actor.get_transform()
                    result[actor_id] = {
                        'type': self.actor_types[actor_id],
                        'location': [transform.location.x, transform.location.y, transform.location.z],
                        'rotation': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
                    }
                return result
                
        except Exception as e:
            logger.error(f"Error getting all actors: {e}")
            return {}
    
    def get_traffic_lights(self) -> List[str]:
        """
        Get all traffic light IDs.
        
        Returns:
            List of traffic light IDs
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return []
                
                traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
                return [str(tl.id) for tl in traffic_lights]
                
        except Exception as e:
            logger.error(f"Error getting traffic lights: {e}")
            return []
    
    def set_traffic_light_state(self, traffic_light_id: str, state: str) -> bool:
        """
        Set traffic light state.
        
        Args:
            traffic_light_id: Traffic light ID
            state: Traffic light state ('Red', 'Yellow', 'Green')
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
                for tl in traffic_lights:
                    if str(tl.id) == traffic_light_id:
                        if state == 'Red':
                            tl.set_state(carla.TrafficLightState.Red)
                        elif state == 'Yellow':
                            tl.set_state(carla.TrafficLightState.Yellow)
                        elif state == 'Green':
                            tl.set_state(carla.TrafficLightState.Green)
                        else:
                            logger.error(f"Invalid traffic light state: {state}")
                            return False
                        
                        logger.info(f"Set traffic light {traffic_light_id} to {state}")
                        return True
                
                logger.error(f"Traffic light {traffic_light_id} not found")
                return False
                
        except Exception as e:
            logger.error(f"Error setting traffic light state: {e}")
            return False
    
    def get_traffic_light_state(self, traffic_light_id: str) -> Optional[str]:
        """
        Get traffic light state.
        
        Args:
            traffic_light_id: Traffic light ID
            
        Returns:
            Traffic light state as string, or None if not found
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return None
                
                traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
                for tl in traffic_lights:
                    if str(tl.id) == traffic_light_id:
                        state = tl.get_state()
                        if state == carla.TrafficLightState.Red:
                            return 'Red'
                        elif state == carla.TrafficLightState.Yellow:
                            return 'Yellow'
                        elif state == carla.TrafficLightState.Green:
                            return 'Green'
                        else:
                            return 'Unknown'
                
                return None
                
        except Exception as e:
            logger.error(f"Error getting traffic light state: {e}")
            return None
    
    def set_traffic_light_timer(self, traffic_light_id: str, time: float) -> bool:
        """
        Set traffic light timer.
        
        Args:
            traffic_light_id: Traffic light ID
            time: Time in seconds
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
                for tl in traffic_lights:
                    if str(tl.id) == traffic_light_id:
                        tl.set_green_time(time)
                        logger.info(f"Set traffic light {traffic_light_id} timer to {time}s")
                        return True
                
                logger.error(f"Traffic light {traffic_light_id} not found")
                return False
                
        except Exception as e:
            logger.error(f"Error setting traffic light timer: {e}")
            return False
    
    def create_sensor(self, sensor_type: str, sensor_id: str,
                     location: List[float], rotation: List[float],
                     attributes: Dict[str, Any] = None) -> bool:
        """
        Create a sensor.
        
        Args:
            sensor_type: Type of sensor
            sensor_id: Unique sensor ID
            location: [x, y, z] coordinates
            rotation: [pitch, yaw, roll] in degrees
            attributes: Sensor attributes
            
        Returns:
            True if creation successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                if sensor_id in self.sensors:
                    logger.warning(f"Sensor {sensor_id} already exists")
                    return False
                
                # Get blueprint
                blueprint = self.world.get_blueprint_library().find(sensor_type)
                if not blueprint:
                    logger.error(f"Sensor blueprint {sensor_type} not found")
                    return False
                
                # Set attributes
                if attributes:
                    for key, value in attributes.items():
                        if blueprint.has_attribute(key):
                            blueprint.set_attribute(key, str(value))
                
                # Create transform
                transform = carla.Transform(
                    carla.Location(location[0], location[1], location[2]),
                    carla.Rotation(rotation[0], rotation[1], rotation[2])
                )
                
                # Create sensor
                sensor = self.world.spawn_actor(blueprint, transform)
                
                # Set up callback
                sensor.listen(lambda data: self._sensor_callback(sensor_id, data))
                
                # Track sensor
                self.sensors[sensor_id] = sensor
                self.sensor_data[sensor_id] = None
                
                logger.info(f"Created sensor {sensor_id} of type {sensor_type}")
                return True
                
        except Exception as e:
            logger.error(f"Error creating sensor {sensor_id}: {e}")
            return False
    
    def destroy_sensor(self, sensor_id: str) -> bool:
        """
        Destroy a sensor.
        
        Args:
            sensor_id: Unique sensor ID
            
        Returns:
            True if destroy successful, False otherwise
        """
        try:
            with self.lock:
                if sensor_id not in self.sensors:
                    logger.warning(f"Sensor {sensor_id} not found")
                    return False
                
                sensor = self.sensors[sensor_id]
                sensor.destroy()
                
                # Remove from tracking
                del self.sensors[sensor_id]
                if sensor_id in self.sensor_data:
                    del self.sensor_data[sensor_id]
                
                logger.info(f"Destroyed sensor {sensor_id}")
                return True
                
        except Exception as e:
            logger.error(f"Error destroying sensor {sensor_id}: {e}")
            return False
    
    def get_sensor_data(self, sensor_id: str) -> Optional[Dict[str, Any]]:
        """
        Get sensor data.
        
        Args:
            sensor_id: Unique sensor ID
            
        Returns:
            Sensor data dictionary, or None if not found
        """
        try:
            with self.lock:
                if sensor_id not in self.sensor_data:
                    return None
                
                return self.sensor_data[sensor_id]
                
        except Exception as e:
            logger.error(f"Error getting sensor data for {sensor_id}: {e}")
            return None
    
    def _sensor_callback(self, sensor_id: str, data):
        """Callback for sensor data."""
        try:
            with self.lock:
                # Convert sensor data to serializable format
                if hasattr(data, 'raw_data'):
                    self.sensor_data[sensor_id] = {
                        'timestamp': data.timestamp,
                        'frame': data.frame,
                        'raw_data': data.raw_data
                    }
                else:
                    self.sensor_data[sensor_id] = {
                        'timestamp': data.timestamp,
                        'frame': data.frame,
                        'data': str(data)
                    }
        except Exception as e:
            logger.error(f"Error in sensor callback for {sensor_id}: {e}")
    
    def _update_sensors(self):
        """Update sensor data."""
        # This is called during simulation step
        # Sensor data is updated via callbacks
        pass
    
    def get_map_name(self) -> str:
        """
        Get current map name.
        
        Returns:
            Current map name
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return ""
                
                return self.world.get_map().name
                
        except Exception as e:
            logger.error(f"Error getting map name: {e}")
            return ""
    
    def get_available_maps(self) -> List[str]:
        """
        Get list of available maps.
        
        Returns:
            List of available map names
        """
        try:
            return self.client.get_available_maps()
        except Exception as e:
            logger.error(f"Error getting available maps: {e}")
            return []
    
    def load_map(self, map_name: str) -> bool:
        """
        Load a map.
        
        Args:
            map_name: Name of the map to load
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                if not self.is_connected():
                    return False
                
                self.world = self.client.load_world(map_name)
                logger.info(f"Loaded map: {map_name}")
                return True
                
        except Exception as e:
            logger.error(f"Error loading map {map_name}: {e}")
            return False
    
    def start(self):
        """Start the XML-RPC server."""
        logger.info(f"Starting CARLA XML-RPC server on {self.host}:{self.port}")
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Server stopped by user")
        finally:
            self.disconnect()
    
    def stop(self):
        """Stop the XML-RPC server."""
        logger.info("Stopping CARLA XML-RPC server")
        self.server.shutdown()
        self.disconnect()


def main():
    """Main function to run the CARLA XML-RPC server."""
    parser = argparse.ArgumentParser(description='CARLA XML-RPC Server for MOSAIC Integration')
    parser.add_argument('--host', default='localhost', help='Host address for XML-RPC server')
    parser.add_argument('--port', type=int, default=8090, help='Port for XML-RPC server')
    parser.add_argument('--carla-host', default='localhost', help='CARLA server host')
    parser.add_argument('--carla-port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Create and start server
    server = CarlaXMLRPCServer(
        host=args.host,
        port=args.port,
        carla_host=args.carla_host,
        carla_port=args.carla_port
    )
    
    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Server interrupted")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
