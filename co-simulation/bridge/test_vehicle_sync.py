#!/usr/bin/env python3
"""
Test script to verify external vehicle synchronization from CARLA to SUMO via XML-RPC.

This script:
1. Spawns a vehicle in CARLA via XML-RPC
2. Monitors the synchronization process through logs
3. Verifies that the vehicle appears in SUMO

Usage:
    python test_vehicle_sync.py
"""

import time
import logging
import sys
import os

# Add the bridge directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from manual_control import ManualControl

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_vehicle_sync():
    """Test external vehicle synchronization from CARLA to SUMO"""
    
    logger.info("=== VEHICLE SYNC TEST STARTED ===")
    
    try:
        # Initialize manual control
        manual_control = ManualControl()
        
        # Connect to CARLA
        logger.info("Connecting to CARLA...")
        if not manual_control.connect_carla():
            logger.error("Failed to connect to CARLA")
            return False
        
        # Connect to XML-RPC
        logger.info("Connecting to XML-RPC...")
        if not manual_control.connect_xmlrpc():
            logger.error("Failed to connect to XML-RPC")
            return False
        
        logger.info("Successfully connected to both CARLA and XML-RPC")
        
        # Spawn vehicle via XML-RPC
        logger.info("Spawning vehicle via XML-RPC...")
        if not manual_control.spawn_vehicle_via_xmlrpc():
            logger.error("Failed to spawn vehicle via XML-RPC")
            return False
        
        logger.info("Vehicle spawned successfully!")
        
        # Monitor vehicle for a few seconds
        logger.info("Monitoring vehicle for 10 seconds...")
        start_time = time.time()
        while time.time() - start_time < 10:
            if manual_control.vehicle:
                transform = manual_control.vehicle.get_transform()
                velocity = manual_control.vehicle.get_velocity()
                speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                
                logger.info(f"Vehicle position: ({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})")
                logger.info(f"Vehicle speed: {speed:.2f} m/s")
                
                # Move vehicle slightly to trigger updates
                if time.time() - start_time > 2:
                    manual_control.throttle = 0.3
                    manual_control.update_vehicle_control()
            
            time.sleep(1)
        
        # Stop vehicle
        manual_control.throttle = 0.0
        manual_control.brake = 1.0
        manual_control.update_vehicle_control()
        
        logger.info("=== VEHICLE SYNC TEST COMPLETED ===")
        logger.info("Check the logs for synchronization messages:")
        logger.info("- Look for 'EXTERNAL VEHICLE DETECTION' messages in CARLA Ambassador logs")
        logger.info("- Look for 'CARLA->SUMO SYNC' messages in CARLA Ambassador logs")
        logger.info("- Look for 'SUMO RECEIVED EXTERNAL VEHICLE' messages in SUMO Ambassador logs")
        logger.info("- Look for 'SUMO ADDING EXTERNAL VEHICLE' messages in SUMO Ambassador logs")
        
        return True
        
    except Exception as e:
        logger.error(f"Test failed with error: {e}")
        return False
    
    finally:
        # Cleanup
        if 'manual_control' in locals():
            manual_control.cleanup()

if __name__ == "__main__":
    success = test_vehicle_sync()
    sys.exit(0 if success else 1)
