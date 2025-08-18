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

import java.net.URL;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

/**
 * Enhanced XML-RPC client for CARLA-MOSAIC integration.
 * 
 * This class provides comprehensive communication with the CARLA XML-RPC server,
 * including actor management, traffic light control, and sensor management.
 * It replaces the TraCI-based bridge with a more suitable XML-RPC architecture.
 */
public class CarlaXmlRpcClient{

    // Sensor-related methods
    private static final String CREATE_SENSOR = "create_sensor";
    private static final String DESTROY_SENSOR = "destroy_sensor";
    private static final String GET_SENSOR_DATA = "get_sensor_data";
    private static final String GET_DETECTED_OBJECTS = "get_detected_objects";
    
    // Connection methods
    private static final String CONNECT = "connect";
    private static final String DISCONNECT = "disconnect";
    private static final String IS_CONNECTED = "is_connected";
    
    // Simulation control methods
    private static final String START_SIMULATION = "start_simulation";
    private static final String STOP_SIMULATION = "stop_simulation";
    private static final String STEP_SIMULATION = "step_simulation";
    private static final String GET_SIMULATION_TIME = "get_simulation_time";
    
    // Actor management methods
    private static final String SPAWN_ACTOR = "spawn_actor";
    private static final String DESTROY_ACTOR = "destroy_actor";
    private static final String UPDATE_ACTOR_TRANSFORM = "update_actor_transform";
    private static final String UPDATE_ACTOR_VELOCITY = "update_actor_velocity";
    private static final String GET_ACTOR_TRANSFORM = "get_actor_transform";
    private static final String GET_ACTOR_VELOCITY = "get_actor_velocity";
    private static final String GET_ALL_ACTORS = "get_all_actors";
    
    // Traffic light management methods
    private static final String GET_TRAFFIC_LIGHTS = "get_traffic_lights";
    private static final String SET_TRAFFIC_LIGHT_STATE = "set_traffic_light_state";
    private static final String GET_TRAFFIC_LIGHT_STATE = "get_traffic_light_state";
    private static final String SET_TRAFFIC_LIGHT_TIMER = "set_traffic_light_timer";
    
    // Map and world information methods
    private static final String GET_MAP_NAME = "get_map_name";
    private static final String GET_AVAILABLE_MAPS = "get_available_maps";
    private static final String LOAD_MAP = "load_map";

    private XmlRpcClient client;
    private final Logger log = LoggerFactory.getLogger(this.getClass());
    private final Gson gson = new Gson();

    /**
     * Constructor for the CARLA XML-RPC client.
     * 
     * @param xmlRpcServerUrl URL of the CARLA XML-RPC server
     */
    public CarlaXmlRpcClient(URL xmlRpcServerUrl) {
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();   
        config.setServerURL(xmlRpcServerUrl);
        // Set reply and connection timeout (both in ms)
        config.setReplyTimeout(6000);
        config.setConnectionTimeout(10000);
        client = new XmlRpcClient();
        client.setConfig(config);
    }

    /**
     * Connect to the CARLA XML-RPC server with retry logic.
     * 
     * @param retryAttempts Number of connection attempts
     * @throws XmlRpcException if connection fails
     * @throws InterruptedException if interrupted during retry
     */
    public void connect(int retryAttempts) throws XmlRpcException, InterruptedException{
        boolean connected = false;
        int currentAttempt = 1;
        while( !connected && retryAttempts >= currentAttempt ) {
            try {
                log.info("Attempting to connect to CARLA XML-RPC server ... ");
                Object[] params = new Object[]{};
                client.execute(CONNECT, params);
                connected = true;
            }
            catch(XmlRpcException e) {
                log.error("Connection attempt {} to connect to CARLA XML-RPC server failed!", currentAttempt, e);
                // Sleep for 1 second between attempts
                Thread.sleep(1000);
                currentAttempt++;
            }
        } 
        if (!connected) {
            throw new XmlRpcException("Failed to connect to XML RPC Server with config " + client.getConfig() + " !");
        }
        log.info("Connected successfully to CARLA XML-RPC server!");    
    }
    
    /**
     * Disconnect from the CARLA XML-RPC server.
     * 
     * @return true if disconnection successful, false otherwise
     * @throws XmlRpcException if disconnection fails
     */
    public boolean disconnect() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Boolean result = (Boolean) client.execute(DISCONNECT, params);
            log.info("Disconnected from CARLA XML-RPC server");
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error disconnecting from CARLA XML-RPC server", e);
            return false;
        }
    }
    
    /**
     * Check if connected to the CARLA XML-RPC server.
     * 
     * @return true if connected, false otherwise
     * @throws XmlRpcException if check fails
     */
    public boolean isConnected() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Boolean result = (Boolean) client.execute(IS_CONNECTED, params);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error checking connection status", e);
            return false;
        }
    }
    
    /**
     * Start the simulation.
     * 
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean startSimulation() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Boolean result = (Boolean) client.execute(START_SIMULATION, params);
            log.info("Simulation started");
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error starting simulation", e);
            return false;
        }
    }
    
    /**
     * Stop the simulation.
     * 
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean stopSimulation() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Boolean result = (Boolean) client.execute(STOP_SIMULATION, params);
            log.info("Simulation stopped");
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error stopping simulation", e);
            return false;
        }
    }
    
    /**
     * Step the simulation by the given time.
     * 
     * @param deltaTime Time step in seconds
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean stepSimulation(double deltaTime) throws XmlRpcException {
        try {
            Object[] params = new Object[]{deltaTime};
            Boolean result = (Boolean) client.execute(STEP_SIMULATION, params);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error stepping simulation", e);
            return false;
        }
    }
    
    /**
     * Get current simulation time.
     * 
     * @return current simulation time in seconds
     * @throws XmlRpcException if operation fails
     */
    public double getSimulationTime() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Double result = (Double) client.execute(GET_SIMULATION_TIME, params);
            return result != null ? result : 0.0;
        } catch (XmlRpcException e) {
            log.error("Error getting simulation time", e);
            return 0.0;
        }
    }
    
    /**
     * Spawn an actor in CARLA.
     * 
     * @param actorType Type of actor (e.g., 'vehicle.tesla.model3')
     * @param actorId Unique identifier for the actor
     * @param location [x, y, z] coordinates
     * @param rotation [pitch, yaw, roll] in degrees
     * @param attributes Additional attributes for the actor
     * @return true if spawn successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean spawnActor(String actorType, String actorId, 
                             List<Double> location, List<Double> rotation,
                             Map<String, Object> attributes) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorType, actorId, location, rotation, attributes};
            Boolean result = (Boolean) client.execute(SPAWN_ACTOR, params);
            log.info("Spawned actor {} of type {}", actorId, actorType);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error spawning actor {}", actorId, e);
            return false;
        }
    }
    
    /**
     * Destroy an actor.
     * 
     * @param actorId Unique identifier for the actor
     * @return true if destroy successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean destroyActor(String actorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorId};
            Boolean result = (Boolean) client.execute(DESTROY_ACTOR, params);
            log.info("Destroyed actor {}", actorId);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error destroying actor {}", actorId, e);
            return false;
        }
    }
    
    /**
     * Update actor transform.
     * 
     * @param actorId Unique identifier for the actor
     * @param location [x, y, z] coordinates
     * @param rotation [pitch, yaw, roll] in degrees
     * @return true if update successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean updateActorTransform(String actorId, List<Double> location, List<Double> rotation) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorId, location, rotation};
            Boolean result = (Boolean) client.execute(UPDATE_ACTOR_TRANSFORM, params);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error updating transform for actor {}", actorId, e);
            return false;
        }
    }
    
    /**
     * Update actor velocity.
     * 
     * @param actorId Unique identifier for the actor
     * @param velocity [x, y, z] velocity components
     * @return true if update successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean updateActorVelocity(String actorId, List<Double> velocity) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorId, velocity};
            Boolean result = (Boolean) client.execute(UPDATE_ACTOR_VELOCITY, params);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error updating velocity for actor {}", actorId, e);
            return false;
        }
    }
    
    /**
     * Get actor transform.
     * 
     * @param actorId Unique identifier for the actor
     * @return Map containing 'location' and 'rotation' lists, or null if not found
     * @throws XmlRpcException if operation fails
     */
    @SuppressWarnings("unchecked")
    public Map<String, List<Double>> getActorTransform(String actorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorId};
            Object result = client.execute(GET_ACTOR_TRANSFORM, params);
            if (result instanceof Map) {
                return (Map<String, List<Double>>) result;
            }
            return null;
        } catch (XmlRpcException e) {
            log.error("Error getting transform for actor {}", actorId, e);
            return null;
        }
    }
    
    /**
     * Get actor velocity.
     * 
     * @param actorId Unique identifier for the actor
     * @return [x, y, z] velocity components, or null if not found
     * @throws XmlRpcException if operation fails
     */
    @SuppressWarnings("unchecked")
    public List<Double> getActorVelocity(String actorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{actorId};
            Object result = client.execute(GET_ACTOR_VELOCITY, params);
            if (result instanceof List) {
                return (List<Double>) result;
            }
            return null;
        } catch (XmlRpcException e) {
            log.error("Error getting velocity for actor {}", actorId, e);
            return null;
        }
    }
    
    /**
     * Get information about all actors.
     * 
     * @return Map mapping actor IDs to actor information
     * @throws XmlRpcException if operation fails
     */
    @SuppressWarnings("unchecked")
    public Map<String, Map<String, Object>> getAllActors() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Object result = client.execute(GET_ALL_ACTORS, params);
            if (result instanceof Map) {
                return (Map<String, Map<String, Object>>) result;
            }
            return new HashMap<>();
        } catch (XmlRpcException e) {
            log.error("Error getting all actors", e);
            return new HashMap<>();
        }
    }
    
    /**
     * Get all traffic light IDs.
     * 
     * @return List of traffic light IDs
     * @throws XmlRpcException if operation fails
     */
    @SuppressWarnings("unchecked")
    public List<String> getTrafficLights() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Object result = client.execute(GET_TRAFFIC_LIGHTS, params);
            if (result instanceof List) {
                return (List<String>) result;
            }
            return Arrays.asList();
        } catch (XmlRpcException e) {
            log.error("Error getting traffic lights", e);
            return Arrays.asList();
        }
    }
    
    /**
     * Set traffic light state.
     * 
     * @param trafficLightId Traffic light ID
     * @param state Traffic light state ('Red', 'Yellow', 'Green')
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean setTrafficLightState(String trafficLightId, String state) throws XmlRpcException {
        try {
            Object[] params = new Object[]{trafficLightId, state};
            Boolean result = (Boolean) client.execute(SET_TRAFFIC_LIGHT_STATE, params);
            log.info("Set traffic light {} to {}", trafficLightId, state);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error setting traffic light state for {}", trafficLightId, e);
            return false;
        }
    }
    
    /**
     * Get traffic light state.
     * 
     * @param trafficLightId Traffic light ID
     * @return Traffic light state as string, or null if not found
     * @throws XmlRpcException if operation fails
     */
    public String getTrafficLightState(String trafficLightId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{trafficLightId};
            Object result = client.execute(GET_TRAFFIC_LIGHT_STATE, params);
            if (result instanceof String) {
                return (String) result;
            }
            return null;
        } catch (XmlRpcException e) {
            log.error("Error getting traffic light state for {}", trafficLightId, e);
            return null;
        }
    }
    
    /**
     * Set traffic light timer.
     * 
     * @param trafficLightId Traffic light ID
     * @param time Time in seconds
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean setTrafficLightTimer(String trafficLightId, double time) throws XmlRpcException {
        try {
            Object[] params = new Object[]{trafficLightId, time};
            Boolean result = (Boolean) client.execute(SET_TRAFFIC_LIGHT_TIMER, params);
            log.info("Set traffic light {} timer to {}s", trafficLightId, time);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error setting traffic light timer for {}", trafficLightId, e);
            return false;
        }
    }
    
    /**
     * Get current map name.
     * 
     * @return Current map name
     * @throws XmlRpcException if operation fails
     */
    public String getMapName() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Object result = client.execute(GET_MAP_NAME, params);
            if (result instanceof String) {
                return (String) result;
            }
            return "";
        } catch (XmlRpcException e) {
            log.error("Error getting map name", e);
            return "";
        }
    }
    
    /**
     * Get list of available maps.
     * 
     * @return List of available map names
     * @throws XmlRpcException if operation fails
     */
    @SuppressWarnings("unchecked")
    public List<String> getAvailableMaps() throws XmlRpcException {
        try {
            Object[] params = new Object[]{};
            Object result = client.execute(GET_AVAILABLE_MAPS, params);
            if (result instanceof List) {
                return (List<String>) result;
            }
            return Arrays.asList();
        } catch (XmlRpcException e) {
            log.error("Error getting available maps", e);
            return Arrays.asList();
        }
    }
    
    /**
     * Load a map.
     * 
     * @param mapName Name of the map to load
     * @return true if successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean loadMap(String mapName) throws XmlRpcException {
        try {
            Object[] params = new Object[]{mapName};
            Boolean result = (Boolean) client.execute(LOAD_MAP, params);
            log.info("Loaded map: {}", mapName);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error loading map {}", mapName, e);
            return false;
        }
    }

    /**
     * Calls CARLA XML-RPC server create_sensor method and logs sensor ID of created sensor.
     * @param registration DetectorRegistration interaction used to create sensor.
     * @throws XmlRpcException if XMLRPC call fails or connection is lost.
     */
    public void createSensor(DetectorRegistration registration) throws XmlRpcException{
        List<Double> location = Arrays.asList(registration.getDetector().getLocation().getX(), registration.getDetector().getLocation().getY(), registration.getDetector().getLocation().getZ());
        List<Double> orientation = Arrays.asList(registration.getDetector().getOrientation().getPitch(), registration.getDetector().getOrientation().getRoll(), registration.getDetector().getOrientation().getYaw());
        Object[] params = new Object[]{registration.getDetector().getSensorId(), registration.getInfrastructureId(), location, orientation, new HashMap<String, Object>()};
        Boolean result = (Boolean) client.execute(CREATE_SENSOR, params);
        if (result != null && result) {
            log.info("Created sensor {} at infrastructure {}", registration.getDetector().getSensorId(), registration.getInfrastructureId());
        } else {
            log.error("Failed to create sensor {} at infrastructure {}", registration.getDetector().getSensorId(), registration.getInfrastructureId());
        }
    }
    
    /**
     * Destroy a sensor.
     * 
     * @param sensorId Unique sensor ID
     * @return true if destroy successful, false otherwise
     * @throws XmlRpcException if operation fails
     */
    public boolean destroySensor(String sensorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{sensorId};
            Boolean result = (Boolean) client.execute(DESTROY_SENSOR, params);
            log.info("Destroyed sensor {}", sensorId);
            return result != null && result;
        } catch (XmlRpcException e) {
            log.error("Error destroying sensor {}", sensorId, e);
            return false;
        }
    }
    
    /**
     * Get sensor data.
     * 
     * @param sensorId Unique sensor ID
     * @return Sensor data as string, or null if not found
     * @throws XmlRpcException if operation fails
     */
    public String getSensorData(String sensorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{sensorId};
            Object result = client.execute(GET_SENSOR_DATA, params);
            if (result instanceof String) {
                return (String) result;
            }
            return null;
        } catch (XmlRpcException e) {
            log.error("Error getting sensor data for {}", sensorId, e);
            return null;
        }
    }
    
    /**
     * Calls CARLA XML-RPC server get_detected_objects method and returns an array of DetectedObject.
     * @param infrastructureId String infrastructure ID of sensor to get detections from.
     * @param sensorId String sensor ID of sensor to get detections from
     * @return DetectedObject[] from given sensor.
     * @throws XmlRpcException if XMLRPC call fails or connection is lost.
     */
    public DetectedObject[] getDetectedObjects(String infrastructureId ,String sensorId) throws XmlRpcException{
        Object[] params = new Object[]{infrastructureId, sensorId};
        Object result = client.execute(GET_DETECTED_OBJECTS, params);
        log.debug("Detections from infrastructure {} sensor {} : {}", infrastructureId, sensorId, result);
        String jsonResult = (String)result;
        return gson.fromJson(jsonResult,DetectedObject[].class);
    }
}
