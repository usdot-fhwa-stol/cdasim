/*
*Copyright (C) 2023 LEIDOS.
*
*Licensed under the Apache License, Version 2.0 (the "License"); you may not
*use this file except in compliance with the License. You may obtain a copy of
*the License at
*
*http://www.apache.org/licenses/LICENSE-2.0
*
*Unless required by applicable law or agreed to in writing, software
*distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
*License for the specific language governing permissions and limitations under
*the License.
*/
package org.eclipse.mosaic.fed.carla;

import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.URL;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Test class for the comprehensive CARLA XML-RPC Client implementation.
 * 
 * This test class demonstrates the new XML-RPC client functionality including:
 * - Connection management with retry logic
 * - Actor discovery and management
 * - Granular data access methods
 * - Traffic light control
 * - Sensor management
 * - Error handling and recovery
 */
public class CarlaXmlRpcClientTest {

    private static final Logger log = LoggerFactory.getLogger(CarlaXmlRpcClientTest.class);
    
    private CarlaXmlRpcClient client;
    private static final String TEST_SERVER_URL = "http://localhost:8090";
    private static final int TEST_RETRY_ATTEMPTS = 3;

    @BeforeEach
    void setUp() throws Exception {
        URL serverUrl = new URL(TEST_SERVER_URL);
        client = new CarlaXmlRpcClient(serverUrl);
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testConnectionManagement() throws Exception {
        log.info("Testing connection management...");
        
        // Test connection
        client.connect(TEST_RETRY_ATTEMPTS);
        assertTrue(client.isConnected(), "Client should be connected after successful connection");
        
        // Test connection status
        boolean isConnected = client.isConnected();
        assertTrue(isConnected, "Connection status should be true");
        
        // Test disconnect
        boolean disconnectResult = client.disconnect();
        assertTrue(disconnectResult, "Disconnect should be successful");
        
        log.info("Connection management test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testSimulationControl() throws Exception {
        log.info("Testing simulation control...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test simulation advancement
        boolean advanceResult = client.advanceSimulation();
        assertTrue(advanceResult, "Simulation advancement should be successful");
        
        // Test simulation time retrieval
        double simulationTime = client.getSimulationTime();
        assertTrue(simulationTime >= 0.0, "Simulation time should be non-negative");
        
        // Test step simulation (backward compatibility)
        boolean stepResult = client.stepSimulation(0.1);
        assertTrue(stepResult, "Step simulation should be successful");
        
        client.disconnect();
        log.info("Simulation control test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testActorDiscovery() throws Exception {
        log.info("Testing actor discovery...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test actor discovery with different filter patterns
        List<Integer> vehicleActors = client.getActiveActorIds("vehicle.*");
        assertNotNull(vehicleActors, "Vehicle actors list should not be null");
        log.info("Found {} vehicle actors", vehicleActors.size());
        
        List<Integer> allActors = client.getActiveActorIds("*");
        assertNotNull(allActors, "All actors list should not be null");
        log.info("Found {} total actors", allActors.size());
        
        // Test actor discovery with null filter (should use default)
        List<Integer> defaultActors = client.getActiveActorIds(null);
        assertNotNull(defaultActors, "Default actors list should not be null");
        
        client.disconnect();
        log.info("Actor discovery test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testActorDataRetrieval() throws Exception {
        log.info("Testing actor data retrieval...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Get active actors
        List<Integer> actorIds = client.getActiveActorIds("vehicle.*");
        if (!actorIds.isEmpty()) {
            Integer testActorId = actorIds.get(0);
            
            // Test basic actor info
            Map<String, Object> basicInfo = client.getActorBasicInfo(testActorId);
            assertNotNull(basicInfo, "Basic actor info should not be null");
            log.info("Actor {} basic info: {}", testActorId, basicInfo);
            
            // Test actor transform
            Map<String, Object> transform = client.getActorTransform(testActorId);
            assertNotNull(transform, "Actor transform should not be null");
            log.info("Actor {} transform: {}", testActorId, transform);
            
            // Test actor velocity
            Map<String, Object> velocity = client.getActorVelocity(testActorId);
            if (velocity != null) {
                log.info("Actor {} velocity: {}", testActorId, velocity);
            }
            
            // Test actor acceleration
            Map<String, Object> acceleration = client.getActorAcceleration(testActorId);
            if (acceleration != null) {
                log.info("Actor {} acceleration: {}", testActorId, acceleration);
            }
            
            // Test actor angular velocity
            Map<String, Object> angularVelocity = client.getActorAngularVelocity(testActorId);
            if (angularVelocity != null) {
                log.info("Actor {} angular velocity: {}", testActorId, angularVelocity);
            }
            
            // Test actor bounding box
            Map<String, Object> boundingBox = client.getActorBoundingBox(testActorId);
            if (boundingBox != null) {
                log.info("Actor {} bounding box: {}", testActorId, boundingBox);
            }
            
            // Test vehicle light state
            Map<String, Object> lightState = client.getVehicleLightState(testActorId);
            if (lightState != null) {
                log.info("Actor {} light state: {}", testActorId, lightState);
            }
        } else {
            log.warn("No actors found for data retrieval test");
        }
        
        client.disconnect();
        log.info("Actor data retrieval test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testActorLifecycle() throws Exception {
        log.info("Testing actor lifecycle...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test actor spawning
        String testActorId = "test_actor_" + System.currentTimeMillis();
        List<Double> location = Arrays.asList(100.0, 200.0, 0.0);
        List<Double> rotation = Arrays.asList(0.0, 90.0, 0.0);
        Map<String, Object> attributes = new HashMap<>();
        attributes.put("role_name", testActorId);
        
        boolean spawnResult = client.spawnActor("vehicle.tesla.model3", testActorId, 
                                               location, rotation, attributes);
        if (spawnResult) {
            log.info("Successfully spawned actor: {}", testActorId);
            
            // Test actor transform update
            List<Double> newLocation = Arrays.asList(150.0, 250.0, 0.0);
            List<Double> newRotation = Arrays.asList(0.0, 180.0, 0.0);
            boolean updateResult = client.updateActorTransform(testActorId, newLocation, newRotation);
            assertTrue(updateResult, "Actor transform update should be successful");
            
            // Test actor velocity update
            List<Double> velocity = Arrays.asList(10.0, 0.0, 0.0);
            boolean velocityResult = client.updateActorVelocity(testActorId, velocity);
            assertTrue(velocityResult, "Actor velocity update should be successful");
            
            // Test comprehensive state update
            Map<String, Object> properties = new HashMap<>();
            Map<String, Object> transform = new HashMap<>();
            Map<String, Object> loc = new HashMap<>();
            loc.put("x", 200.0);
            loc.put("y", 300.0);
            loc.put("z", 0.0);
            transform.put("location", loc);
            properties.put("transform", transform);
            
            boolean stateResult = client.setActorStateProperties(testActorId, properties);
            assertTrue(stateResult, "Actor state properties update should be successful");
            
            // Test actor destruction
            boolean destroyResult = client.destroyActor(testActorId);
            assertTrue(destroyResult, "Actor destruction should be successful");
            log.info("Successfully destroyed actor: {}", testActorId);
        } else {
            log.warn("Failed to spawn test actor - skipping lifecycle test");
        }
        
        client.disconnect();
        log.info("Actor lifecycle test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testTrafficLightManagement() throws Exception {
        log.info("Testing traffic light management...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test getting all traffic light states
        List<Map<String, Object>> trafficLightStates = client.getAllTrafficLightStates();
        assertNotNull(trafficLightStates, "Traffic light states should not be null");
        log.info("Found {} traffic lights", trafficLightStates.size());
        
        if (!trafficLightStates.isEmpty()) {
            Map<String, Object> firstTrafficLight = trafficLightStates.get(0);
            Object trafficLightId = firstTrafficLight.get("id");
            
            if (trafficLightId != null) {
                // Test individual traffic light state retrieval
                Map<String, Object> state = client.getTrafficLightState(trafficLightId);
                assertNotNull(state, "Traffic light state should not be null");
                log.info("Traffic light {} state: {}", trafficLightId, state);
                
                // Test traffic light state setting
                boolean setStateResult = client.setTrafficLightState(trafficLightId.toString(), "Red");
                assertTrue(setStateResult, "Traffic light state setting should be successful");
                
                // Test traffic light timer setting
                boolean setTimerResult = client.setTrafficLightTimer(trafficLightId.toString(), 30.0);
                assertTrue(setTimerResult, "Traffic light timer setting should be successful");
            }
        } else {
            log.warn("No traffic lights found for management test");
        }
        
        client.disconnect();
        log.info("Traffic light management test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testSensorManagement() throws Exception {
        log.info("Testing sensor management...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test sensor creation
        String testSensorId = "test_sensor_" + System.currentTimeMillis();
        List<Double> location = Arrays.asList(0.0, 0.0, 2.0);
        List<Double> rotation = Arrays.asList(0.0, 0.0, 0.0);
        Map<String, Object> attributes = new HashMap<>();
        attributes.put("fov", "90");
        
        boolean createResult = client.createSensor("sensor.camera.rgb", testSensorId, 
                                                 location, rotation, attributes);
        if (createResult) {
            log.info("Successfully created sensor: {}", testSensorId);
            
            // Test sensor data retrieval
            Map<String, Object> sensorData = client.getSensorData(testSensorId);
            if (sensorData != null) {
                log.info("Sensor {} data: {}", testSensorId, sensorData);
            }
            
            // Test detected objects retrieval
            try {
                var detectedObjects = client.getDetectedObjects("", testSensorId);
                log.info("Sensor {} detected {} objects", testSensorId, 
                        detectedObjects != null ? detectedObjects.length : 0);
            } catch (Exception e) {
                log.warn("Failed to get detected objects: {}", e.getMessage());
            }
            
            // Test sensor destruction
            boolean destroyResult = client.destroySensor(testSensorId);
            assertTrue(destroyResult, "Sensor destruction should be successful");
            log.info("Successfully destroyed sensor: {}", testSensorId);
        } else {
            log.warn("Failed to create test sensor - skipping sensor management test");
        }
        
        client.disconnect();
        log.info("Sensor management test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testMapManagement() throws Exception {
        log.info("Testing map management...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test current map name retrieval
        String currentMap = client.getMapName();
        assertNotNull(currentMap, "Current map name should not be null");
        log.info("Current map: {}", currentMap);
        
        // Test available maps retrieval
        List<String> availableMaps = client.getAvailableMaps();
        assertNotNull(availableMaps, "Available maps list should not be null");
        log.info("Available maps: {}", availableMaps);
        
        // Test map loading (if available maps exist)
        if (!availableMaps.isEmpty()) {
            String testMap = availableMaps.get(0);
            boolean loadResult = client.loadMap(testMap);
            assertTrue(loadResult, "Map loading should be successful");
            log.info("Successfully loaded map: {}", testMap);
        } else {
            log.warn("No available maps found for loading test");
        }
        
        client.disconnect();
        log.info("Map management test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testErrorHandling() throws Exception {
        log.info("Testing error handling...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Test with invalid actor ID
        Map<String, Object> invalidActorInfo = client.getActorBasicInfo(999999);
        assertNull(invalidActorInfo, "Invalid actor should return null");
        
        // Test with invalid traffic light ID
        Map<String, Object> invalidTrafficLightState = client.getTrafficLightState(999999);
        assertNull(invalidTrafficLightState, "Invalid traffic light should return null");
        
        // Test with invalid sensor ID
        Map<String, Object> invalidSensorData = client.getSensorData(999999);
        assertNull(invalidSensorData, "Invalid sensor should return null");
        
        // Test actor destruction with invalid ID
        boolean invalidDestroyResult = client.destroyActor(999999);
        assertFalse(invalidDestroyResult, "Invalid actor destruction should fail");
        
        client.disconnect();
        log.info("Error handling test completed successfully");
    }

    @Test
    @Disabled("Requires running CARLA XML-RPC server")
    void testComprehensiveIntegration() throws Exception {
        log.info("Testing comprehensive integration...");
        
        client.connect(TEST_RETRY_ATTEMPTS);
        
        // Simulate a complete tick-loop integration as per requirements
        log.info("Starting comprehensive integration test...");
        
        // 1. Discover Actors (As Needed)
        List<Integer> activeActorIds = client.getActiveActorIds("vehicle.*");
        log.info("Discovered {} active vehicle actors", activeActorIds.size());
        
        // 2. Update CARLA Actors (Optional) - simulate external data synchronization
        if (!activeActorIds.isEmpty()) {
            Integer testActorId = activeActorIds.get(0);
            Map<String, Object> properties = new HashMap<>();
            Map<String, Object> transform = new HashMap<>();
            Map<String, Object> location = new HashMap<>();
            location.put("x", 100.0);
            location.put("y", 200.0);
            location.put("z", 0.0);
            transform.put("location", location);
            properties.put("transform", transform);
            
            boolean syncResult = client.setActorStateProperties(testActorId, properties);
            log.info("Actor synchronization result: {}", syncResult);
        }
        
        // 3. Step CARLA
        boolean advanceResult = client.advanceSimulation();
        assertTrue(advanceResult, "Simulation advancement should be successful");
        
        // 4. Retrieve Needed CARLA State
        for (Integer actorId : activeActorIds) {
            Map<String, Object> transform = client.getActorTransform(actorId);
            Map<String, Object> velocity = client.getActorVelocity(actorId);
            Map<String, Object> acceleration = client.getActorAcceleration(actorId);
            
            if (transform != null) {
                log.info("Actor {} data retrieved successfully", actorId);
            }
        }
        
        // 5. Retrieve traffic light states
        List<Map<String, Object>> trafficLightStates = client.getAllTrafficLightStates();
        log.info("Retrieved {} traffic light states", trafficLightStates.size());
        
        // 6. Process sensor data (if any sensors exist)
        // This would typically involve getting sensor data and processing it
        
        client.disconnect();
        log.info("Comprehensive integration test completed successfully");
    }
}
