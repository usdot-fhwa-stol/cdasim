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
package org.eclipse.mosaic.fed.carla.carlaconnect;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.apache.xmlrpc.XmlRpcException;

import java.net.URL;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/**
 * Unit tests for the CARLA XML-RPC client.
 * 
 * Tests the actor management and traffic light control functionality
 * of the XML-RPC client implementation.
 */
@ExtendWith(MockitoExtension.class)
class CarlaXmlRpcClientTest {

    @Mock
    private XmlRpcClient mockXmlRpcClient;

    private CarlaXmlRpcClient carlaClient;
    private URL testServerUrl;

    @BeforeEach
    void setUp() throws Exception {
        testServerUrl = new URL("http://localhost:8000");
        carlaClient = new CarlaXmlRpcClient(testServerUrl);
        
        // Use reflection to inject the mock client for testing
        // Note: In a real implementation, you might want to use a different approach
        // such as dependency injection or a factory pattern
    }

    @Test
    void testConnect() throws Exception {
        // Test successful connection
        when(mockXmlRpcClient.execute(eq("connect"), any(Object[].class)))
            .thenReturn(true);
        
        assertDoesNotThrow(() -> carlaClient.connect(3));
    }

    @Test
    void testConnectFailure() {
        // Test connection failure
        when(mockXmlRpcClient.execute(eq("connect"), any(Object[].class)))
            .thenThrow(new XmlRpcException("Connection failed"));
        
        assertThrows(XmlRpcException.class, () -> carlaClient.connect(1));
    }

    @Test
    void testDisconnect() throws Exception {
        when(mockXmlRpcClient.execute(eq("disconnect"), any(Object[].class)))
            .thenReturn(true);
        
        boolean result = carlaClient.disconnect();
        assertTrue(result);
    }

    @Test
    void testIsConnected() throws Exception {
        when(mockXmlRpcClient.execute(eq("is_connected"), any(Object[].class)))
            .thenReturn(true);
        
        boolean result = carlaClient.isConnected();
        assertTrue(result);
    }

    @Test
    void testStartSimulation() throws Exception {
        when(mockXmlRpcClient.execute(eq("start_simulation"), any(Object[].class)))
            .thenReturn(true);
        
        boolean result = carlaClient.startSimulation();
        assertTrue(result);
    }

    @Test
    void testStopSimulation() throws Exception {
        when(mockXmlRpcClient.execute(eq("stop_simulation"), any(Object[].class)))
            .thenReturn(true);
        
        boolean result = carlaClient.stopSimulation();
        assertTrue(result);
    }

    @Test
    void testStepSimulation() throws Exception {
        when(mockXmlRpcClient.execute(eq("step_simulation"), any(Object[].class)))
            .thenReturn(true);
        
        boolean result = carlaClient.stepSimulation(0.1);
        assertTrue(result);
    }

    @Test
    void testGetSimulationTime() throws Exception {
        when(mockXmlRpcClient.execute(eq("get_simulation_time"), any(Object[].class)))
            .thenReturn(10.5);
        
        double result = carlaClient.getSimulationTime();
        assertEquals(10.5, result, 0.001);
    }

    @Test
    void testSpawnActor() throws Exception {
        when(mockXmlRpcClient.execute(eq("spawn_actor"), any(Object[].class)))
            .thenReturn(true);
        
        String actorType = "vehicle.tesla.model3";
        String actorId = "test_vehicle_1";
        List<Double> location = Arrays.asList(100.0, 200.0, 0.0);
        List<Double> rotation = Arrays.asList(0.0, 90.0, 0.0);
        Map<String, Object> attributes = new HashMap<>();
        attributes.put("role_name", "test_vehicle");
        
        boolean result = carlaClient.spawnActor(actorType, actorId, location, rotation, attributes);
        assertTrue(result);
    }

    @Test
    void testDestroyActor() throws Exception {
        when(mockXmlRpcClient.execute(eq("destroy_actor"), any(Object[].class)))
            .thenReturn(true);
        
        String actorId = "test_vehicle_1";
        boolean result = carlaClient.destroyActor(actorId);
        assertTrue(result);
    }

    @Test
    void testUpdateActorTransform() throws Exception {
        when(mockXmlRpcClient.execute(eq("update_actor_transform"), any(Object[].class)))
            .thenReturn(true);
        
        String actorId = "test_vehicle_1";
        List<Double> location = Arrays.asList(110.0, 210.0, 0.0);
        List<Double> rotation = Arrays.asList(0.0, 95.0, 0.0);
        
        boolean result = carlaClient.updateActorTransform(actorId, location, rotation);
        assertTrue(result);
    }

    @Test
    void testUpdateActorVelocity() throws Exception {
        when(mockXmlRpcClient.execute(eq("update_actor_velocity"), any(Object[].class)))
            .thenReturn(true);
        
        String actorId = "test_vehicle_1";
        List<Double> velocity = Arrays.asList(10.0, 0.0, 0.0);
        
        boolean result = carlaClient.updateActorVelocity(actorId, velocity);
        assertTrue(result);
    }

    @Test
    void testGetActorTransform() throws Exception {
        Map<String, List<Double>> expectedTransform = new HashMap<>();
        expectedTransform.put("location", Arrays.asList(100.0, 200.0, 0.0));
        expectedTransform.put("rotation", Arrays.asList(0.0, 90.0, 0.0));
        
        when(mockXmlRpcClient.execute(eq("get_actor_transform"), any(Object[].class)))
            .thenReturn(expectedTransform);
        
        String actorId = "test_vehicle_1";
        Map<String, List<Double>> result = carlaClient.getActorTransform(actorId);
        
        assertNotNull(result);
        assertEquals(expectedTransform.get("location"), result.get("location"));
        assertEquals(expectedTransform.get("rotation"), result.get("rotation"));
    }

    @Test
    void testGetActorVelocity() throws Exception {
        List<Double> expectedVelocity = Arrays.asList(10.0, 0.0, 0.0);
        
        when(mockXmlRpcClient.execute(eq("get_actor_velocity"), any(Object[].class)))
            .thenReturn(expectedVelocity);
        
        String actorId = "test_vehicle_1";
        List<Double> result = carlaClient.getActorVelocity(actorId);
        
        assertNotNull(result);
        assertEquals(expectedVelocity, result);
    }

    @Test
    void testGetAllActors() throws Exception {
        Map<String, Map<String, Object>> expectedActors = new HashMap<>();
        Map<String, Object> actorInfo = new HashMap<>();
        actorInfo.put("type", "vehicle.tesla.model3");
        actorInfo.put("transform", new HashMap<String, List<Double>>());
        expectedActors.put("test_vehicle_1", actorInfo);
        
        when(mockXmlRpcClient.execute(eq("get_all_actors"), any(Object[].class)))
            .thenReturn(expectedActors);
        
        Map<String, Map<String, Object>> result = carlaClient.getAllActors();
        
        assertNotNull(result);
        assertEquals(expectedActors.size(), result.size());
        assertTrue(result.containsKey("test_vehicle_1"));
    }

    @Test
    void testGetTrafficLights() throws Exception {
        List<String> expectedTrafficLights = Arrays.asList("traffic_light_1", "traffic_light_2");
        
        when(mockXmlRpcClient.execute(eq("get_traffic_lights"), any(Object[].class)))
            .thenReturn(expectedTrafficLights);
        
        List<String> result = carlaClient.getTrafficLights();
        
        assertNotNull(result);
        assertEquals(expectedTrafficLights, result);
    }

    @Test
    void testSetTrafficLightState() throws Exception {
        when(mockXmlRpcClient.execute(eq("set_traffic_light_state"), any(Object[].class)))
            .thenReturn(true);
        
        String trafficLightId = "traffic_light_1";
        String state = "Red";
        
        boolean result = carlaClient.setTrafficLightState(trafficLightId, state);
        assertTrue(result);
    }

    @Test
    void testGetTrafficLightState() throws Exception {
        String expectedState = "Green";
        
        when(mockXmlRpcClient.execute(eq("get_traffic_light_state"), any(Object[].class)))
            .thenReturn(expectedState);
        
        String trafficLightId = "traffic_light_1";
        String result = carlaClient.getTrafficLightState(trafficLightId);
        
        assertNotNull(result);
        assertEquals(expectedState, result);
    }

    @Test
    void testSetTrafficLightTimer() throws Exception {
        when(mockXmlRpcClient.execute(eq("set_traffic_light_timer"), any(Object[].class)))
            .thenReturn(true);
        
        String trafficLightId = "traffic_light_1";
        double time = 30.0;
        
        boolean result = carlaClient.setTrafficLightTimer(trafficLightId, time);
        assertTrue(result);
    }

    @Test
    void testGetMapName() throws Exception {
        String expectedMapName = "Town01";
        
        when(mockXmlRpcClient.execute(eq("get_map_name"), any(Object[].class)))
            .thenReturn(expectedMapName);
        
        String result = carlaClient.getMapName();
        
        assertNotNull(result);
        assertEquals(expectedMapName, result);
    }

    @Test
    void testGetAvailableMaps() throws Exception {
        List<String> expectedMaps = Arrays.asList("Town01", "Town02", "Town03");
        
        when(mockXmlRpcClient.execute(eq("get_available_maps"), any(Object[].class)))
            .thenReturn(expectedMaps);
        
        List<String> result = carlaClient.getAvailableMaps();
        
        assertNotNull(result);
        assertEquals(expectedMaps, result);
    }

    @Test
    void testLoadMap() throws Exception {
        when(mockXmlRpcClient.execute(eq("load_map"), any(Object[].class)))
            .thenReturn(true);
        
        String mapName = "Town02";
        boolean result = carlaClient.loadMap(mapName);
        assertTrue(result);
    }

    @Test
    void testDestroySensor() throws Exception {
        when(mockXmlRpcClient.execute(eq("destroy_sensor"), any(Object[].class)))
            .thenReturn(true);
        
        String sensorId = "test_sensor_1";
        boolean result = carlaClient.destroySensor(sensorId);
        assertTrue(result);
    }

    @Test
    void testGetSensorData() throws Exception {
        String expectedData = "{\"detections\": []}";
        
        when(mockXmlRpcClient.execute(eq("get_sensor_data"), any(Object[].class)))
            .thenReturn(expectedData);
        
        String sensorId = "test_sensor_1";
        String result = carlaClient.getSensorData(sensorId);
        
        assertNotNull(result);
        assertEquals(expectedData, result);
    }

    @Test
    void testErrorHandling() throws Exception {
        // Test that exceptions are properly handled and logged
        when(mockXmlRpcClient.execute(anyString(), any(Object[].class)))
            .thenThrow(new XmlRpcException("Test error"));
        
        // These should return false/null instead of throwing exceptions
        assertFalse(carlaClient.disconnect());
        assertFalse(carlaClient.isConnected());
        assertFalse(carlaClient.startSimulation());
        assertFalse(carlaClient.stopSimulation());
        assertFalse(carlaClient.stepSimulation(0.1));
        assertEquals(0.0, carlaClient.getSimulationTime(), 0.001);
        assertFalse(carlaClient.spawnActor("test", "test", Arrays.asList(0.0, 0.0, 0.0), 
                                         Arrays.asList(0.0, 0.0, 0.0), new HashMap<>()));
        assertFalse(carlaClient.destroyActor("test"));
        assertFalse(carlaClient.updateActorTransform("test", Arrays.asList(0.0, 0.0, 0.0), 
                                                    Arrays.asList(0.0, 0.0, 0.0)));
        assertFalse(carlaClient.updateActorVelocity("test", Arrays.asList(0.0, 0.0, 0.0)));
        assertNull(carlaClient.getActorTransform("test"));
        assertNull(carlaClient.getActorVelocity("test"));
        assertTrue(carlaClient.getAllActors().isEmpty());
        assertTrue(carlaClient.getTrafficLights().isEmpty());
        assertFalse(carlaClient.setTrafficLightState("test", "Red"));
        assertNull(carlaClient.getTrafficLightState("test"));
        assertFalse(carlaClient.setTrafficLightTimer("test", 30.0));
        assertEquals("", carlaClient.getMapName());
        assertTrue(carlaClient.getAvailableMaps().isEmpty());
        assertFalse(carlaClient.loadMap("test"));
        assertFalse(carlaClient.destroySensor("test"));
        assertNull(carlaClient.getSensorData("test"));
    }
}
