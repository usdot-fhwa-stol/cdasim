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
Automatic Vehicle Control Script for CARLA-SUMO Co-simulation

This script creates an automatically controlled vehicle in CARLA that drives at a constant
speed to test position synchronization with SUMO. When the script stops, the vehicle is 
properly removed from CARLA, and the CARLA Ambassador will detect the removal and send 
VehicleUpdates with removedNames to SUMO.

Usage:
    python manual_control.py [--carla-host localhost] [--carla-port 2000] [--xmlrpc-host localhost] [--xmlrpc-port 8090]

The vehicle will automatically drive at a constant speed (30% throttle) in a straight line.
Press Ctrl+C to stop the vehicle and exit the script.
"""

import argparse
import sys
import os
import time
import threading
import signal
import logging
import math
from typing import Optional, Dict, Any
import xmlrpc.client

# Add CARLA Python API to path
try:
    import glob
    sys.path.append(
        glob.glob('PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Cannot find CARLA library .egg file")
    sys.exit(1)

import carla

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ManualControl")

class ManualControl:
    def __init__(self, carla_host='localhost', carla_port=2000, xmlrpc_host='localhost', xmlrpc_port=8090):
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.xmlrpc_host = xmlrpc_host
        self.xmlrpc_port = xmlrpc_port
        
        # CARLA connection
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.vehicle: Optional[carla.Vehicle] = None
        self.vehicle_id: Optional[str] = None
        
        # XML-RPC connection for bridge communication
        self.xmlrpc_client: Optional[xmlrpc.client.ServerProxy] = None
        
        # Control state
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        
        # Vehicle control parameters - set to constant values for automatic driving
        self.steer = 0.0  # Straight ahead
        self.throttle = 0.3  # Constant moderate speed
        self.brake = 0.0
        self.reverse = False
        
        # Movement parameters
        self.max_steer_angle = 0.7
        self.max_throttle = 1.0
        self.max_brake = 1.0
        self.steer_speed = 3.0
        self.throttle_speed = 3.0
        self.brake_speed = 3.0
        
        # Auto-driving parameters
        self.auto_driving = True
        self.target_speed = 0.3  # 30% throttle for reasonable speed
        self.steering_adjustment = 0.0  # No steering adjustment for straight driving
        
        # Position tracking for synchronization testing
        self.last_position = None
        self.position_update_interval = 0.1  # seconds
        self.last_position_update = 0.0
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logger.info(f"Received signal {signum}, shutting down...")
        self.cleanup()
        sys.exit(0)

    def connect_carla(self) -> bool:
        """Connect to CARLA server"""
        try:
            self.client = carla.Client(self.carla_host, self.carla_port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            logger.info(f"Connected to CARLA at {self.carla_host}:{self.carla_port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to CARLA: {e}")
            return False

    def connect_xmlrpc(self) -> bool:
        """Connect to XML-RPC bridge server"""
        try:
            self.xmlrpc_client = xmlrpc.client.ServerProxy(
                f"http://{self.xmlrpc_host}:{self.xmlrpc_port}", 
                allow_none=True
            )
            # Test connection
            if self.xmlrpc_client.is_connected():
                logger.info(f"Connected to XML-RPC bridge at {self.xmlrpc_host}:{self.xmlrpc_port}")
                return True
            else:
                logger.warning("XML-RPC bridge not connected to CARLA")
                return False
        except Exception as e:
            logger.error(f"Failed to connect to XML-RPC bridge: {e}")
            return False

    def spawn_vehicle(self) -> bool:
        """Spawn a controllable vehicle in CARLA"""
        try:
            # Use fixed spawn position (200, -169) with proper ground height
            spawn_location = carla.Location(x=200.0, y=-169.0, z=0.0)
            
            # Get the ground height at this location
            waypoint = self.world.get_map().get_waypoint(spawn_location)
            if waypoint:
                spawn_location.z = waypoint.transform.location.z + 0.5  # Add small offset above ground
            else:
                # Fallback: try to find a nearby spawn point
                spawn_points = self.world.get_map().get_spawn_points()
                if spawn_points:
                    # Find the closest spawn point to our target location
                    target_location = carla.Location(x=200.0, y=-169.0, z=0.0)
                    closest_point = min(spawn_points, 
                                      key=lambda p: p.location.distance(target_location))
                    spawn_location = closest_point.location
                    spawn_location.x = 200.0  # Keep our target X coordinate
                    spawn_location.y = -169.0  # Keep our target Y coordinate
                    spawn_location.z += 0.5  # Add small offset above ground
                else:
                    spawn_location.z = 1.0  # Default height if no waypoints found
            
            spawn_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            spawn_point = carla.Transform(spawn_location, spawn_rotation)
            
            # Get vehicle blueprint
            blueprint_library = self.world.get_blueprint_library()
            vehicle_blueprint = blueprint_library.find('vehicle.tesla.model3')
            if not vehicle_blueprint:
                # Try alternative vehicles
                for vehicle_type in ['vehicle.audi.tt', 'vehicle.lincoln.mkz_2017', 'vehicle.nissan.micra']:
                    vehicle_blueprint = blueprint_library.find(vehicle_type)
                    if vehicle_blueprint:
                        break
                
                if not vehicle_blueprint:
                    logger.error("No suitable vehicle blueprint found")
                    return False
            
            # Set vehicle attributes
            vehicle_blueprint.set_attribute('role_name', 'manual_control_vehicle')
            
            # Try to spawn the vehicle with collision detection
            self.vehicle = self.world.try_spawn_actor(vehicle_blueprint, spawn_point)
            if not self.vehicle:
                # If spawn failed due to collision, try nearby positions
                logger.warning(f"Failed to spawn at exact position ({spawn_location.x}, {spawn_location.y}, {spawn_location.z}), trying nearby positions...")
                
                # Try spawning at nearby positions
                offsets = [(0, 2), (0, -2), (2, 0), (-2, 0), (2, 2), (-2, -2), (2, -2), (-2, 2)]
                for offset_x, offset_y in offsets:
                    try_location = carla.Location(
                        x=spawn_location.x + offset_x,
                        y=spawn_location.y + offset_y,
                        z=spawn_location.z
                    )
                    try_waypoint = self.world.get_map().get_waypoint(try_location)
                    if try_waypoint:
                        try_location.z = try_waypoint.transform.location.z + 0.5
                    
                    try_point = carla.Transform(try_location, spawn_rotation)
                    self.vehicle = self.world.try_spawn_actor(vehicle_blueprint, try_point)
                    if self.vehicle:
                        logger.info(f"Successfully spawned vehicle at nearby position: ({try_location.x}, {try_location.y}, {try_location.z})")
                        break
                
                if not self.vehicle:
                    logger.error("Failed to spawn vehicle at any nearby position")
                    return False
            
            self.vehicle_id = str(self.vehicle.id)
            logger.info(f"Spawned vehicle with ID: {self.vehicle_id} at position: {spawn_point.location}")
            
            # Set initial position for tracking
            self.last_position = self.vehicle.get_transform()
            self.last_position_update = time.time()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to spawn vehicle: {e}")
            return False

    def spawn_vehicle_via_xmlrpc(self) -> bool:
        """Spawn vehicle via XML-RPC bridge (alternative method)"""
        try:
            if not self.xmlrpc_client:
                logger.error("XML-RPC client not connected")
                return False
            
            # Use fixed spawn position (200, -169) with proper ground height
            spawn_location = carla.Location(x=200.0, y=-169.0, z=0.0)
            
            # Get the ground height at this location
            waypoint = self.world.get_map().get_waypoint(spawn_location)
            if waypoint:
                spawn_location.z = waypoint.transform.location.z + 0.5
            else:
                spawn_location.z = 1.0  # Default height
            
            location = [spawn_location.x, spawn_location.y, spawn_location.z]
            rotation = [0.0, 0.0, 0.0]
            
            # Spawn via XML-RPC
            success = self.xmlrpc_client.spawn_actor(
                'vehicle.tesla.model3',
                'manual_control_vehicle',
                location,
                rotation,
                {'role_name': 'manual_control_vehicle'}
            )
            
            if success:
                logger.info("Vehicle spawned via XML-RPC bridge")
                # Get the vehicle from CARLA world
                for actor in self.world.get_actors():
                    if hasattr(actor, 'attributes') and actor.attributes.get('role_name') == 'manual_control_vehicle':
                        self.vehicle = actor
                        self.vehicle_id = str(actor.id)
                        logger.info(f"Found spawned vehicle with ID: {self.vehicle_id}")
                        return True
            
            logger.error("Failed to spawn vehicle via XML-RPC")
            return False
            
        except Exception as e:
            logger.error(f"Failed to spawn vehicle via XML-RPC: {e}")
            return False

    def update_vehicle_control(self):
        """Update vehicle control based on current input state"""
        if not self.vehicle:
            return
        
        try:
            # Create control command
            control = carla.VehicleControl()
            control.steer = self.steer
            control.throttle = self.throttle
            control.brake = self.brake
            control.reverse = self.reverse
            control.hand_brake = False
            control.manual_gear_shift = False
            
            # Apply control
            self.vehicle.apply_control(control)
            
        except Exception as e:
            logger.error(f"Failed to apply vehicle control: {e}")

    def update_position_tracking(self):
        """Update position tracking for synchronization testing"""
        if not self.vehicle:
            return
        
        current_time = time.time()
        if current_time - self.last_position_update >= self.position_update_interval:
            try:
                current_transform = self.vehicle.get_transform()
                current_velocity = self.vehicle.get_velocity()
                
                # Log position changes for synchronization testing
                if self.last_position:
                    distance = self.last_position.location.distance(current_transform.location)
                    if distance > 0.1:  # Only log significant movements
                        logger.info(f"Vehicle moved {distance:.2f}m to position: "
                                  f"({current_transform.location.x:.2f}, "
                                  f"{current_transform.location.y:.2f}, "
                                  f"{current_transform.location.z:.2f})")
                        # Calculate velocity magnitude using math
                        velocity_magnitude = math.sqrt(current_velocity.x**2 + current_velocity.y**2 + current_velocity.z**2)
                        logger.info(f"Velocity: {velocity_magnitude:.2f} m/s")
                
                self.last_position = current_transform
                self.last_position_update = current_time
                
            except Exception as e:
                logger.error(f"Failed to update position tracking: {e}")

    def control_loop(self):
        """Main control loop - automatic driving mode"""
        logger.info("Starting automatic control loop...")
        logger.info("Vehicle will drive automatically at constant speed")
        logger.info("Press Ctrl+C to stop")
        
        while self.running:
            try:
                # Update vehicle control with constant values
                self.update_vehicle_control()
                
                # Update position tracking
                self.update_position_tracking()
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                time.sleep(0.1)


    def _reset_vehicle_position(self):
        """Reset vehicle to spawn point"""
        if not self.vehicle:
            return
        
        try:
            # Reset to fixed spawn position (200, -169) with proper ground height
            spawn_location = carla.Location(x=200.0, y=-169.0, z=0.0)
            
            # Get the ground height at this location
            waypoint = self.world.get_map().get_waypoint(spawn_location)
            if waypoint:
                spawn_location.z = waypoint.transform.location.z + 0.5
            else:
                spawn_location.z = 1.0  # Default height
            
            spawn_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            spawn_point = carla.Transform(spawn_location, spawn_rotation)
            self.vehicle.set_transform(spawn_point)
            logger.info(f"Vehicle position reset to ({spawn_location.x}, {spawn_location.y}, {spawn_location.z})")
        except Exception as e:
            logger.error(f"Failed to reset vehicle position: {e}")

    def destroy_vehicle(self):
        """Destroy the controlled vehicle"""
        if self.vehicle:
            try:
                vehicle_id = self.vehicle_id
                self.vehicle.destroy()
                logger.info(f"Destroyed vehicle with ID: {vehicle_id}")
                
                # Give some time for the destruction to be processed
                time.sleep(0.5)
                
                # The CARLA Ambassador should detect this removal and send VehicleUpdates
                # with removedNames to SUMO
                logger.info("Vehicle removed from CARLA. CARLA Ambassador should detect this and send VehicleUpdates with removedNames to SUMO.")
                
            except Exception as e:
                logger.error(f"Failed to destroy vehicle: {e}")
            finally:
                self.vehicle = None
                self.vehicle_id = None

    def destroy_vehicle_via_xmlrpc(self):
        """Destroy vehicle via XML-RPC bridge"""
        if self.vehicle_id and self.xmlrpc_client:
            try:
                success = self.xmlrpc_client.destroy_actor(self.vehicle_id)
                if success:
                    logger.info(f"Destroyed vehicle via XML-RPC: {self.vehicle_id}")
                else:
                    logger.warning(f"Failed to destroy vehicle via XML-RPC: {self.vehicle_id}")
            except Exception as e:
                logger.error(f"Error destroying vehicle via XML-RPC: {e}")

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up...")
        self.running = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        # Destroy vehicle
        self.destroy_vehicle()
        
        # Try XML-RPC destruction as backup
        self.destroy_vehicle_via_xmlrpc()
        
        # Close local XML-RPC client proxy without disconnecting the server
        if self.xmlrpc_client:
            try:
                if hasattr(self.xmlrpc_client, 'close'):
                    self.xmlrpc_client.close()
            except Exception:
                pass
            finally:
                self.xmlrpc_client = None
        
        logger.info("Cleanup completed")

    def run(self):
        """Main run method"""
        logger.info("Starting Automatic Vehicle Control for CARLA-SUMO Co-simulation")
        
        # Connect to CARLA
        if not self.connect_carla():
            logger.error("Failed to connect to CARLA")
            return False
        
        # Connect to XML-RPC bridge (optional)
        self.connect_xmlrpc()
        
        # Spawn vehicle directly via CARLA (default method)
        if not self.spawn_vehicle():
            logger.warning("Failed to spawn vehicle directly via CARLA, trying XML-RPC fallback...")
            # Fallback to XML-RPC method if direct CARLA spawning fails
            if not self.spawn_vehicle_via_xmlrpc():
                logger.error("Failed to spawn vehicle via both direct CARLA and XML-RPC methods")
                return False
        
        # Start control loop
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
        
        try:
            # Wait for control thread to finish
            self.control_thread.join()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.cleanup()
        
            return True

def main():
    parser = argparse.ArgumentParser(description='Automatic Vehicle Control for CARLA-SUMO Co-simulation')
    parser.add_argument('--carla-host', default='localhost', help='CARLA server host')
    parser.add_argument('--carla-port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--xmlrpc-host', default='localhost', help='XML-RPC bridge host')
    parser.add_argument('--xmlrpc-port', type=int, default=8090, help='XML-RPC bridge port')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Create and run manual control
    manual_control = ManualControl(
        carla_host=args.carla_host,
        carla_port=args.carla_port,
        xmlrpc_host=args.xmlrpc_host,
        xmlrpc_port=args.xmlrpc_port
    )
    
    success = manual_control.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
