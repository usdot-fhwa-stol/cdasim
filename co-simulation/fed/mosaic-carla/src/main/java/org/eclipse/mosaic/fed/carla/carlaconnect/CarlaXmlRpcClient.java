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
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

/**
 * Comprehensive XML-RPC Client for CARLA Server Integration
 * 
 * This client implements all XML-RPC methods available in the CARLA XML-RPC server,
 * providing robust error handling, retry logic, and comprehensive actor management
 * for MOSAIC co-simulation integration.
 */
public class CarlaXmlRpcClient {

    private static final Logger log = LoggerFactory.getLogger(CarlaXmlRpcClient.class);
    
    // XML-RPC method names
    private static final String CONNECT = "connect";
    private static final String DISCONNECT = "disconnect";
    private static final String IS_CONNECTED = "is_connected";
    
    // Simulation control
    private static final String ADVANCE_SIMULATION = "advance_simulation";
    private static final String STEP_SIMULATION = "step_simulation";
    private static final String GET_SIMULATION_TIME = "get_simulation_time";
    
    // Actor discovery and management
    private static final String GET_ACTIVE_ACTOR_IDS = "get_active_actor_ids";
    private static final String GET_ACTOR_BASIC_INFO = "get_actor_basic_info";
    private static final String GET_ACTOR_TRANSFORM = "get_actor_transform";
    private static final String GET_ACTOR_VELOCITY = "get_actor_velocity";
    private static final String GET_ACTOR_ACCELERATION = "get_actor_acceleration";
    private static final String GET_ACTOR_ANGULAR_VELOCITY = "get_actor_angular_velocity";
    private static final String GET_ACTOR_BOUNDING_BOX = "get_actor_bounding_box";
    private static final String GET_VEHICLE_LIGHT_STATE = "get_vehicle_light_state";
    private static final String SET_ACTOR_STATE_PROPERTIES = "set_actor_state_properties";
    
    // Actor lifecycle
    private static final String SPAWN_ACTOR = "spawn_actor";
    private static final String DESTROY_ACTOR = "destroy_actor";
    private static final String UPDATE_ACTOR_TRANSFORM = "update_actor_transform";
    private static final String UPDATE_ACTOR_VELOCITY = "update_actor_velocity";
    private static final String GET_ALL_ACTORS = "get_all_actors";
    
    // Traffic lights
    private static final String GET_TRAFFIC_LIGHT_STATE = "get_traffic_light_state";
    private static final String GET_ALL_TRAFFIC_LIGHT_STATES = "get_all_traffic_light_states";
    private static final String SET_TRAFFIC_LIGHT_STATE = "set_traffic_light_state";
    private static final String SET_TRAFFIC_LIGHT_TIMER = "set_traffic_light_timer";
    private static final String FREEZE_ALL_TRAFFIC_LIGHTS = "freeze_all_traffic_lights";
    
    // Sensors
    private static final String GET_DETECTED_OBJECTS = "get_detected_objects";
    
    // Maps
    private static final String GET_MAP_NAME = "get_map_name";
    private static final String GET_AVAILABLE_MAPS = "get_available_maps";
    private static final String LOAD_MAP = "load_map";

    // Coordinate transform configuration
    private static final String SET_NET_OFFSET_XY = "set_net_offset_xy";

    // V2X Communication
    private static final String SEND_V2X_MESSAGE = "send_v2x_message";

    // Configuration
    private static final int DEFAULT_RETRY_ATTEMPTS = 3;
    private static final long DEFAULT_RETRY_DELAY_MS = 1000;
    private static final int DEFAULT_REPLY_TIMEOUT_MS = 10000;
    private static final int DEFAULT_CONNECTION_TIMEOUT_MS = 15000;
    
    // Performance optimization constants
    private static final double LOCATION_TOLERANCE = 0.01; // 1cm tolerance for location changes
    private static final double VELOCITY_TOLERANCE = 0.1;  // 0.1 m/s tolerance for velocity changes
    private static final int MAX_RETRY_DELAY_MULTIPLIER = 16; // Cap exponential backoff

    private XmlRpcClient client;
    private final URL serverUrl;
    private final AtomicInteger requestCounter = new AtomicInteger(0);
    private final Gson gson = new Gson();
    
    // Connection state
    private volatile boolean isConnected = false;
    private final Object connectionLock = new Object();
    
    // State management for change detection
    private final Map<String, ActorState> previousActorStates = new ConcurrentHashMap<>();
    private final Map<String, Map<String, Object>> previousTrafficLightStates = new ConcurrentHashMap<>();
    
    // Server type for identification
    public enum ServerType {
        SENSOR_LIB, ACTOR_LIB
    }
    
    private final ServerType serverType;

    /**
     * Constructor for CARLA XML-RPC Client
     * @param xmlRpcServerUrl URL of the CARLA XML-RPC server
     * @param serverType Type of server (SENSOR_LIB or ACTOR_LIB)
     */
    public CarlaXmlRpcClient(URL xmlRpcServerUrl, ServerType serverType) {
        this.serverUrl = xmlRpcServerUrl;
        this.serverType = serverType;
        initializeClient();
    }
    
    /**
     * Constructor for CARLA XML-RPC Client (backward compatibility)
     * @param xmlRpcServerUrl URL of the CARLA XML-RPC server
     */
    public CarlaXmlRpcClient(URL xmlRpcServerUrl) {
        this(xmlRpcServerUrl, ServerType.ACTOR_LIB); // Default to ACTOR_LIB for backward compatibility
    }

    /**
     * Initialize the XML-RPC client with configuration
     */
    private void initializeClient() {
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();   
        config.setServerURL(serverUrl);
        config.setReplyTimeout(DEFAULT_REPLY_TIMEOUT_MS);
        config.setConnectionTimeout(DEFAULT_CONNECTION_TIMEOUT_MS);
        config.setEnabledForExtensions(true);
        
        client = new XmlRpcClient();
        client.setConfig(config);
        
        log.info("CARLA XML-RPC client initialized for {} server: {}", serverType, serverUrl);
    }
    
    /**
     * Get the server type for this client
     * @return ServerType enum value
     */
    public ServerType getServerType() {
        return serverType;
    }

    /**
     * Connect to the CARLA XML-RPC server with retry logic
     * @param retryAttempts Number of retry attempts
     * @throws XmlRpcException if connection fails after all retries
     * @throws InterruptedException if interrupted during retry delays
     */
    public void connect(int retryAttempts) throws XmlRpcException, InterruptedException {
        synchronized (connectionLock) {
            if (isConnected) {
                log.debug("Already connected to CARLA server");
                return;
            }
            
            boolean connected = false;
            int currentAttempt = 1;
            
            while (!connected && retryAttempts >= currentAttempt) {
                try {
                    log.info("Attempting to connect to CARLA XML-RPC server (attempt {}/{})", currentAttempt, retryAttempts);
                    Object[] params = new Object[]{};
                    Object result = executeWithRetry(CONNECT, params, DEFAULT_RETRY_ATTEMPTS);
                    
                    if (result instanceof Boolean && (Boolean) result) {
                        connected = true;
                        isConnected = true;
                        log.info("Successfully connected to CARLA XML-RPC server");

                        // Input frame configuration removed on server; assuming CARLA-frame inputs
                    } else {
                        log.warn("Connection attempt {} returned unexpected result: {}", currentAttempt, result);
                    }
                } catch (XmlRpcException e) {
                    log.error("Connection attempt {} failed: {}", currentAttempt, e.getMessage());
                    if (currentAttempt < retryAttempts) {
                        Thread.sleep(DEFAULT_RETRY_DELAY_MS);
                    }
                    currentAttempt++;
                }
            }
            
            if (!connected) {
                throw new XmlRpcException("Failed to connect to CARLA XML-RPC server after " + retryAttempts + " attempts");
            }
        }
    }

    // Input frame mode configuration removed; client now assumes CARLA-frame inputs

    /**
     * Configure server SUMO net offset (x, y) applied before transform.
     * @param x offset x
     * @param y offset y
     * @return true if accepted
     */
    public boolean setNetOffsetXY(double x, double y) {
        try {
            Object[] params = new Object[]{x, y};
            Object result = executeWithRetry(SET_NET_OFFSET_XY, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to set net offset ({}, {}): {}", x, y, e.getMessage());
            return false;
        }
    }

    /**
     * Disconnect from the CARLA XML-RPC server
     * @return true if disconnected successfully
     */
    public boolean disconnect() {
        synchronized (connectionLock) {
            if (!isConnected) {
                log.debug("Already disconnected from CARLA server");
                return true;
            }
            
            try {
                Object[] params = new Object[]{};
                Object result = executeWithRetry(DISCONNECT, params, DEFAULT_RETRY_ATTEMPTS);
                
                if (result instanceof Boolean && (Boolean) result) {
                    isConnected = false;
                    log.info("Successfully disconnected from CARLA XML-RPC server");
                    return true;
                } else {
                    log.warn("Disconnect returned unexpected result: {}", result);
                    return false;
                }
            } catch (Exception e) {
                log.error("Error during disconnect: {}", e.getMessage());
                isConnected = false; // Force disconnect state
                return false;
            }
        }
    }

    /**
     * Check if connected to the CARLA server
     * @return true if connected
     */
    public boolean isConnected() {
        synchronized (connectionLock) {
            // First check local state
            if (!isConnected) {
                return false;
            }
            
            // Then verify with server
            try {
                Object[] params = new Object[]{};
                Object result = executeWithRetry(IS_CONNECTED, params, 1); // Use minimal retries for status check
                boolean serverConnected = result instanceof Boolean && (Boolean) result;
                
                // Update local state if server disagrees
                if (!serverConnected && isConnected) {
                    log.warn("Server reports disconnected, updating local state");
                    isConnected = false;
                }
                
                return serverConnected;
            } catch (Exception e) {
                log.debug("Connection check failed: {}", e.getMessage());
                // Don't update local state on network errors, just return false
                return false;
            }
        }
    }

    /**
     * Advance the CARLA simulation by one tick
     * @return true if successful
     */
    public boolean advanceSimulation() {
        try {
            Object[] params = new Object[]{};
            Object result = executeWithRetry(ADVANCE_SIMULATION, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to advance simulation: {}", e.getMessage());
            return false;
        }
    }

    /**
     * Step the CARLA simulation (backward compatibility)
     * @param deltaTime Time step in seconds
     * @return true if successful
     */
    public boolean stepSimulation(double deltaTime) {
        try {
            Object[] params = new Object[]{deltaTime};
            Object result = executeWithRetry(STEP_SIMULATION, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to step simulation: {}", e.getMessage());
            return false;
        }
    }

    /**
     * Get current simulation time
     * @return simulation time in seconds
     */
    public double getSimulationTime() {
        try {
            Object[] params = new Object[]{};
            Object result = executeWithRetry(GET_SIMULATION_TIME, params, DEFAULT_RETRY_ATTEMPTS);
            if (result instanceof Number) {
                return ((Number) result).doubleValue();
            }
            return 0.0;
        } catch (Exception e) {
            log.error("Failed to get simulation time: {}", e.getMessage());
            return 0.0;
        }
    }

    /**
     * Get active actor IDs with optional filter pattern
     * @param filterPattern Filter pattern (e.g., "vehicle.*")
     * @return List of actor IDs
     */
    @SuppressWarnings("unchecked")
    public List<Integer> getActiveActorIds(String filterPattern) {
        try {
            Object[] params = new Object[]{filterPattern != null ? filterPattern : "vehicle.*"};
            Object result = executeWithRetry(GET_ACTIVE_ACTOR_IDS, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof List) {
                List<Integer> actorIds = new ArrayList<>();
                for (Object item : (List<?>) result) {
                    if (item instanceof Number) {
                        actorIds.add(((Number) item).intValue());
                    }
                }
                return actorIds;
            }
            return new ArrayList<>();
        } catch (Exception e) {
            log.error("Failed to get active actor IDs: {}", e.getMessage());
            return new ArrayList<>();
        }
    }

    /**
     * Get basic information about an actor
     * @param actorKey Actor ID or name
     * @return Actor basic info as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorBasicInfo(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_BASIC_INFO, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor basic info for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get actor transform (position and rotation)
     * @param actorKey Actor ID or name
     * @return Transform data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorTransform(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_TRANSFORM, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor transform for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get actor velocity
     * @param actorKey Actor ID or name
     * @return Velocity data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorVelocity(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_VELOCITY, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor velocity for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get actor acceleration
     * @param actorKey Actor ID or name
     * @return Acceleration data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorAcceleration(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_ACCELERATION, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor acceleration for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get actor angular velocity
     * @param actorKey Actor ID or name
     * @return Angular velocity data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorAngularVelocity(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_ANGULAR_VELOCITY, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor angular velocity for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get actor bounding box
     * @param actorKey Actor ID or name
     * @return Bounding box data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorBoundingBox(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_ACTOR_BOUNDING_BOX, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get actor bounding box for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Get vehicle light state
     * @param actorKey Actor ID or name
     * @return Light state data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getVehicleLightState(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(GET_VEHICLE_LIGHT_STATE, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get vehicle light state for {}: {}", actorKey, e.getMessage());
            return null;
        }
    }

    /**
     * Set actor state properties (transform, velocity, etc.)
     * @param actorKey Actor ID or name
     * @param properties Properties to set
     * @return true if successful
     */
    public boolean setActorStateProperties(Object actorKey, Map<String, Object> properties) {
        try {
            Object[] params = new Object[]{actorKey, properties};
            Object result = executeWithRetry(SET_ACTOR_STATE_PROPERTIES, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to set actor state properties for {}: {}", actorKey, e.getMessage());
            return false;
        }
    }

    /**
     * Spawn an actor in CARLA
     * @param actorType Type of actor to spawn
     * @param actorId Unique ID for the actor
     * @param location Location [x, y, z]
     * @param rotation Rotation [pitch, yaw, roll]
     * @param attributes Additional attributes
     * @return CARLA internal actor ID if successful, null otherwise
     */
    public String spawnActor(String actorType, String actorId, List<Double> location, 
                             List<Double> rotation, Map<String, Object> attributes) {
        try {
            log.info("XML-RPC spawn_actor call: type={}, id={}, location={}, rotation={}, attributes={}", 
                    actorType, actorId, location, rotation, attributes);
            
            Object[] params = new Object[]{actorType, actorId, location, rotation, attributes != null ? attributes : new HashMap<>()};
            Object result = executeWithRetry(SPAWN_ACTOR, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof String) {
                String carlaId = (String) result;
                log.info("XML-RPC spawn_actor result: CARLA ID={} (String)", carlaId);
                return carlaId;
            } else if (result instanceof Number) {
                // Handle integer ID from server
                String carlaId = String.valueOf(result);
                log.info("XML-RPC spawn_actor result: CARLA ID={} (converted from {})", carlaId, result.getClass().getSimpleName());
                return carlaId;
            } else if (result instanceof Boolean && (Boolean) result) {
                // Fallback: if server still returns boolean true, return the actorId as the internal ID
                log.info("XML-RPC spawn_actor result: boolean true, using actorId as CARLA ID");
                return actorId;
            } else {
                log.warn("XML-RPC spawn_actor result: unexpected type {} with value {}", 
                        result != null ? result.getClass().getSimpleName() : "null", result);
                return null;
            }
        } catch (Exception e) {
            log.error("Failed to spawn actor {} of type {}: {}", actorId, actorType, e.getMessage());
            return null;
        }
    }

    /**
     * Destroy an actor
     * @param actorKey Actor ID or name
     * @return true if successful
     */
    public boolean destroyActor(Object actorKey) {
        try {
            Object[] params = new Object[]{actorKey};
            Object result = executeWithRetry(DESTROY_ACTOR, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to destroy actor {}: {}", actorKey, e.getMessage());
            return false;
        }
    }

    /**
     * Update actor transform
     * @param actorKey Actor ID or name
     * @param location New location [x, y, z]
     * @param rotation New rotation [pitch, yaw, roll]
     * @return true if successful
     */
    public boolean updateActorTransform(Object actorKey, List<Double> location, List<Double> rotation) {
        try {
            Object[] params = new Object[]{actorKey, location, rotation};
            Object result = executeWithRetry(UPDATE_ACTOR_TRANSFORM, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to update actor transform for {}: {}", actorKey, e.getMessage());
            return false;
        }
    }

    /**
     * Update actor velocity
     * @param actorKey Actor ID or name
     * @param velocity New velocity [x, y, z]
     * @return true if successful
     */
    public boolean updateActorVelocity(Object actorKey, List<Double> velocity) {
        try {
            Object[] params = new Object[]{actorKey, velocity};
            Object result = executeWithRetry(UPDATE_ACTOR_VELOCITY, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to update actor velocity for {}: {}", actorKey, e.getMessage());
            return false;
        }
    }

    /**
     * Get all actors with their basic information
     * @return Map of actor ID to actor information
     */
    @SuppressWarnings("unchecked")
    public Map<String, Map<String, Object>> getAllActors() {
        try {
            Object[] params = new Object[]{};
            Object result = executeWithRetry(GET_ALL_ACTORS, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Map<String, Object>>) result;
            }
            return new HashMap<>();
        } catch (Exception e) {
            log.error("Failed to get all actors: {}", e.getMessage());
            return new HashMap<>();
        }
    }

    /**
     * Get all actors excluding SUMO-managed vehicles
     * @param sumoToCarlaMapping Mapping of SUMO vehicle IDs to CARLA internal IDs to exclude
     * @return Map of actor ID to actor information (excluding SUMO-managed vehicles)
     */
    @SuppressWarnings("unchecked")
    public Map<String, Map<String, Object>> getAllActorsExcludingSumo(Map<String, String> sumoToCarlaMapping) {
        try {
            Map<String, Map<String, Object>> allActors = getAllActors();
            Map<String, Map<String, Object>> filteredActors = new HashMap<>();
            
            if (sumoToCarlaMapping == null || sumoToCarlaMapping.isEmpty()) {
                log.debug("getAllActorsExcludingSumo: No SUMO-to-CARLA mapping provided, returning all actors");
                return allActors;
            }
            
            // Filter out SUMO-managed vehicles
            for (Map.Entry<String, Map<String, Object>> entry : allActors.entrySet()) {
                String actorId = entry.getKey();
                if (!sumoToCarlaMapping.containsValue(actorId)) {
                    filteredActors.put(actorId, entry.getValue());
                } else {
                    log.debug("Excluding SUMO-managed actor '{}' from getAllActors result", actorId);
                }
            }
            
            log.debug("getAllActorsExcludingSumo: {} total actors, {} after filtering SUMO vehicles", 
                     allActors.size(), filteredActors.size());
            return filteredActors;
        } catch (Exception e) {
            log.error("Failed to get actors excluding SUMO: {}", e.getMessage());
            return new HashMap<>();
        }
    }

    /**
     * Get traffic light state
     * @param trafficLightId Traffic light ID
     * @return Traffic light state data as Map, or null if failed
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getTrafficLightState(Object trafficLightId) {
        try {
            Object[] params = new Object[]{trafficLightId};
            Object result = executeWithRetry(GET_TRAFFIC_LIGHT_STATE, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof Map) {
                return (Map<String, Object>) result;
            }
            return null;
        } catch (Exception e) {
            log.error("Failed to get traffic light state for {}: {}", trafficLightId, e.getMessage());
            return null;
        }
    }

    /**
     * Get all traffic light states
     * @return List of traffic light state data
     */
    @SuppressWarnings("unchecked")
    public List<Map<String, Object>> getAllTrafficLightStates() {
        try {
            Object result = executeWithRetry(GET_ALL_TRAFFIC_LIGHT_STATES, new Object[]{}, DEFAULT_RETRY_ATTEMPTS);

            List<Map<String, Object>> states = new ArrayList<>();

            if (result instanceof Object[]) {
                for (Object item : (Object[]) result) {
                    if (item instanceof Map) {
                        states.add((Map<String, Object>) item);
                    }
                }
                return states;
            }

            if (result instanceof List) {
                for (Object item : (List<?>) result) {
                    if (item instanceof Map) {
                        states.add((Map<String, Object>) item);
                    }
                }
                return states;
            }

            log.warn("getAllTrafficLightStates: unexpected result type {}", 
                    (result == null ? "null" : result.getClass().getName()));
            return states;

        } catch (Exception e) {
            log.error("Failed to get all traffic light states: {}", e.getMessage(), e);
            return new ArrayList<>();
        }
    }

    /**
     * Set traffic light state
     * @param trafficLightId Traffic light ID
     * @param state New state ("Red", "Yellow", "Green")
     * @return true if successful
     */
    public boolean setTrafficLightState(Object trafficLightId, String state) {
        try {
            Object[] params = new Object[]{trafficLightId, state};
            Object result = executeWithRetry(SET_TRAFFIC_LIGHT_STATE, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to set traffic light state for {} to {}: {}", trafficLightId, state, e.getMessage());
            return false;
        }
    }

    /**
     * Set traffic light timer
     * @param trafficLightId Traffic light ID
     * @param timeSeconds Time in seconds
     * @return true if successful
     */
    public boolean setTrafficLightTimer(Object trafficLightId, double timeSeconds) {
        try {
            Object[] params = new Object[]{trafficLightId, timeSeconds};
            Object result = executeWithRetry(SET_TRAFFIC_LIGHT_TIMER, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to set traffic light timer for {} to {}s: {}", trafficLightId, timeSeconds, e.getMessage());
            return false;
        }
    }

    public int freezeAllTrafficLights(boolean frozen) {
        try {
            Object result = executeWithRetry(FREEZE_ALL_TRAFFIC_LIGHTS, new Object[]{frozen}, DEFAULT_RETRY_ATTEMPTS);
            if (result instanceof Integer) return (Integer) result;
            if (result instanceof Number)  return ((Number) result).intValue();
            return 0;
        } catch (Exception e) {
            log.error("Failed to freeze-all TLs ({}): {}", frozen, e.getMessage());
            return 0;
        }
    }

    /**
     * Send V2X message to CARLA simulator via XML-RPC
     * @param receiverId ID of the receiving entity in CARLA
     * @param message V2X message content
     * @param senderId ID of the sending entity
     * @param timestamp Message timestamp
     * @return true if successful
     */
    public boolean sendV2xMessage(String receiverId, String message, String senderId, Long timestamp) {
        try {
            Map<String, Object> messageData = new HashMap<>();
            messageData.put("receiverId", receiverId);
            messageData.put("message", message);
            messageData.put("senderId", senderId);
            messageData.put("timestamp", timestamp);
            
            Object[] params = new Object[]{messageData};
            Object result = executeWithRetry(SEND_V2X_MESSAGE, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to send V2X message to {}: {}", receiverId, e.getMessage());
            return false;
        }
    }

    /**
     * Get detected objects from sensor (backward compatibility)
     * @param infrastructureId Infrastructure ID
     * @param sensorId Sensor ID
     * @return Array of detected objects
     * @throws XmlRpcException if retrieval fails
     */
    public DetectedObject[] getDetectedObjects(String infrastructureId, String sensorId) throws XmlRpcException {
        try {
            Object[] params = new Object[]{infrastructureId, sensorId};
            Object result = executeWithRetry(GET_DETECTED_OBJECTS, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof String) {
                String jsonResult = (String) result;
                return gson.fromJson(jsonResult, DetectedObject[].class);
            }
            return new DetectedObject[0];
        } catch (Exception e) {
            log.error("Failed to get detected objects from sensor {}: {}", sensorId, e.getMessage());
            throw new XmlRpcException("Failed to get detected objects", e);
        }
    }

    /**
     * Get current map name
     * @return Map name, or empty string if failed
     */
    public String getMapName() {
        try {
            Object[] params = new Object[]{};
            Object result = executeWithRetry(GET_MAP_NAME, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof String) {
                return (String) result;
            }
            return "";
        } catch (Exception e) {
            log.error("Failed to get map name: {}", e.getMessage());
            return "";
        }
    }

    /**
     * Get available maps
     * @return List of available map names
     */
    @SuppressWarnings("unchecked")
    public List<String> getAvailableMaps() {
        try {
            Object[] params = new Object[]{};
            Object result = executeWithRetry(GET_AVAILABLE_MAPS, params, DEFAULT_RETRY_ATTEMPTS);
            
            if (result instanceof List) {
                List<String> maps = new ArrayList<>();
                for (Object item : (List<?>) result) {
                    if (item instanceof String) {
                        maps.add((String) item);
                    }
                }
                return maps;
            }
            return new ArrayList<>();
        } catch (Exception e) {
            log.error("Failed to get available maps: {}", e.getMessage());
            return new ArrayList<>();
        }
    }

    /**
     * Load a map
     * @param mapName Name of the map to load
     * @return true if successful
     */
    public boolean loadMap(String mapName) {
        try {
            Object[] params = new Object[]{mapName};
            Object result = executeWithRetry(LOAD_MAP, params, DEFAULT_RETRY_ATTEMPTS);
            return result instanceof Boolean && (Boolean) result;
        } catch (Exception e) {
            log.error("Failed to load map {}: {}", mapName, e.getMessage());
            return false;
        }
    }

    /**
     * Execute XML-RPC call with retry logic and connection recovery
     * @param methodName Name of the XML-RPC method
     * @param params Method parameters
     * @param maxRetries Maximum number of retry attempts
     * @return Method result
     * @throws XmlRpcException if all retries fail
     */
    private Object executeWithRetry(String methodName, Object[] params, int maxRetries) throws XmlRpcException {
        int attempt = 0;
        Exception lastException = null;
        
        while (attempt < maxRetries) {
            try {
                int requestId = requestCounter.incrementAndGet();
                // log.debug("Executing XML-RPC call {} (request #{})", methodName, requestId);
                Object result = client.execute(methodName, params);
                // log.debug("XML-RPC call {} completed successfully (request #{})", methodName, requestId);
                return result;
                
            } catch (XmlRpcException e) {
                lastException = e;
                attempt++;
                
                // Check if this is a connection-related error
                boolean isConnectionError = isConnectionError(e);
                if (isConnectionError) {
                    synchronized (connectionLock) {
                        isConnected = false;
                        log.warn("Connection lost during XML-RPC call {}, marking as disconnected", methodName);
                    }
                }
                
                if (attempt < maxRetries) {
                    log.warn("XML-RPC call {} failed (attempt {}/{}): {}", methodName, attempt, maxRetries, e.getMessage());
                    try {
                        // Use exponential backoff for retries
                        long delay = DEFAULT_RETRY_DELAY_MS * (1L << Math.min(attempt - 1, 4)); // Cap at MAX_RETRY_DELAY_MULTIPLIER
                        Thread.sleep(delay);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        throw new XmlRpcException("Interrupted during retry", ie);
                    }
                } else {
                    log.error("XML-RPC call {} failed after {} attempts", methodName, maxRetries);
                }
            }
        }
        
        throw new XmlRpcException("Failed after " + maxRetries + " attempts", lastException);
    }
    
    /**
     * Check if an exception indicates a connection problem
     */
    private boolean isConnectionError(XmlRpcException e) {
        String message = e.getMessage();
        if (message == null) return false;
        
        String lowerMessage = message.toLowerCase();
        return lowerMessage.contains("connection") || 
               lowerMessage.contains("timeout") || 
               lowerMessage.contains("refused") ||
               lowerMessage.contains("unreachable") ||
               lowerMessage.contains("broken pipe");
    }

    /**
     * Start CARLA simulation (backward compatibility)
     * @return true if successful
     */
    public boolean startSimulation() {
        // In the unified design, simulation is controlled by advance_simulation()
        return isConnected();
    }

    /**
     * Stop CARLA simulation (backward compatibility)
     * @return true if successful
     */
    public boolean stopSimulation() {
        // In the unified design, simulation is controlled by advance_simulation()
        return true;
    }

    /**
     * Get traffic lights (backward compatibility)
     * @return List of traffic light IDs
     */
    public List<String> getTrafficLights() {
        try {
            List<Map<String, Object>> states = getAllTrafficLightStates();
            List<String> trafficLightIds = new ArrayList<>();
            
            for (Map<String, Object> state : states) {
                Object id = state.get("id");
                if (id != null) {
                    trafficLightIds.add(id.toString());
                }
            }
            
            return trafficLightIds;
        } catch (Exception e) {
            log.error("Failed to get traffic lights: {}", e.getMessage());
            return new ArrayList<>();
        }
    }

    /**
     * Get actor changes since last call (added, updated, removed)
     * This method provides high-level change detection functionality
     * @param sumoToCarlaMapping Mapping of SUMO vehicle IDs to CARLA internal IDs to exclude from changes
     * @return Map containing "added", "updated", "removed" lists
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getActorChanges(Map<String, String> sumoToCarlaMapping) {
        Map<String, Object> changes = new HashMap<>();
        List<Map<String, Object>> added = new ArrayList<>();
        List<Map<String, Object>> updated = new ArrayList<>();
        List<String> removed = new ArrayList<>();
        
        try {
            // Get current actors excluding SUMO-managed vehicles
            Map<String, Map<String, Object>> currentActors = getAllActorsExcludingSumo(sumoToCarlaMapping);
            log.debug("getActorChanges: Retrieved {} current actors (excluding SUMO-managed)", currentActors.size());
            
            // Convert current actors to ActorState and cache them
            Map<String, ActorState> currentActorStates = new HashMap<>();
            
            // Find added and updated actors
            for (Map.Entry<String, Map<String, Object>> entry : currentActors.entrySet()) {
                String actorId = entry.getKey();
                Map<String, Object> currentStateMap = new HashMap<>(entry.getValue());
                
                // Add actor ID to the state map so it can be retrieved later
                currentStateMap.put("id", actorId);
                
                // Skip actors that are already managed by SUMO (exclude from changes)
                if (sumoToCarlaMapping != null && sumoToCarlaMapping.containsValue(actorId)) {
                    log.debug("Skipping actor {} as it's managed by SUMO", actorId);
                    continue;
                }
                
                // Convert to typed ActorState
                ActorState currentState = ActorState.fromMap(actorId, currentStateMap);
                if (currentState == null) {
                    log.warn("Failed to convert actor state to ActorState for actor {}", actorId);
                    continue;
                }
                
                currentActorStates.put(actorId, currentState);
                
                if (!previousActorStates.containsKey(actorId)) {
                    // New actor
                    log.debug("New actor detected: {}", actorId);
                    added.add(currentStateMap);
                } else {
                    // Existing actor - check for changes
                    ActorState previousState = previousActorStates.get(actorId);
                    updated.add(currentStateMap);
                    log.debug("Actor {} updated", actorId);
                    
                }
            }
            
            // Find removed actors
            for (String previousActorId : previousActorStates.keySet()) {
                // Skip actors that are managed by SUMO
                if (sumoToCarlaMapping != null && sumoToCarlaMapping.containsValue(previousActorId)) {
                    log.debug("Skipping removal check for actor {} as it's managed by SUMO", previousActorId);
                    continue;
                }
                
                if (!currentActorStates.containsKey(previousActorId)) {
                    removed.add(previousActorId);
                }
            }
            log.debug("getActorChanges: Found added={}, updated={}, removed={}", added.size(), updated.size(), removed.size());
            if (!added.isEmpty() || !updated.isEmpty() || !removed.isEmpty()) {
                log.info("Actor changes: added={}, updated={}, removed={}", added.size(), updated.size(), removed.size());
            } 
            
            // Update cache with typed ActorState objects
            previousActorStates.clear();
            previousActorStates.putAll(currentActorStates);
            
            changes.put("added", added);
            changes.put("updated", updated);
            changes.put("removed", removed);
            
        } catch (Exception e) {
            log.error("Failed to get actor changes: {}", e.getMessage(), e);
        }
        
        return changes;
    }

    /**
     * Get traffic light changes since last call
     * @return Map of changed traffic light states
     */
    @SuppressWarnings("unchecked")
    public Map<String, Map<String, Object>> getTrafficLightChanges() {
        Map<String, Map<String, Object>> changes = new HashMap<>();
        
        try {
            List<Map<String, Object>> currentTlStates = getAllTrafficLightStates();
            
            for (Map<String, Object> tl : currentTlStates) {
                Object id = tl.get("id");
                Object state = tl.get("state");
                Object timer = tl.get("timer");
                String idStr = id != null ? id.toString() : null;
                String stateStr = state != null ? state.toString() : null;
                Double timerVal = null;
                if (timer instanceof Number) timerVal = ((Number) timer).doubleValue();
                
                if (idStr != null && stateStr != null) {
                    // Check if traffic light state changed
                    boolean hasChanged = true;
                    if (previousTrafficLightStates.containsKey(idStr)) {
                        Map<String, Object> prevState = previousTrafficLightStates.get(idStr);
                        String prevStateStr = prevState.get("state") != null ? prevState.get("state").toString() : null;
                        Double prevTimer = prevState.get("timer") instanceof Number ? ((Number) prevState.get("timer")).doubleValue() : null;
                        hasChanged = !stateStr.equals(prevStateStr) || 
                                   (timerVal != null && prevTimer != null && !timerVal.equals(prevTimer));
                    }
                    
                    if (hasChanged) {
                        Map<String, Object> tlInfo = new HashMap<>();
                        tlInfo.put("id", idStr);
                        tlInfo.put("state", stateStr);
                        tlInfo.put("timer", timerVal);
                        changes.put(idStr, tlInfo);
                    }
                }
            }
            
            // Update traffic light cache
            previousTrafficLightStates.clear();
            for (Map<String, Object> tl : currentTlStates) {
                String id = tl.get("id") != null ? tl.get("id").toString() : null;
                if (id != null) {
                    previousTrafficLightStates.put(id, new HashMap<>(tl));
                }
            }
            
        } catch (Exception e) {
            log.error("Failed to get traffic light changes: {}", e.getMessage());
        }
        
        return changes;
    }

    /**
     * Check if actor state has changed between two states
     * @param previousState Previous actor state
     * @param currentState Current actor state
     * @return true if state has changed
     */
    private boolean hasActorStateChanged(ActorState previousState, ActorState currentState) {
        if (previousState == null || currentState == null) {
            return true;
        }
        
        // Quick reference equality check first
        if (previousState == currentState) {
            return false;
        }
        
        String actorId = currentState.getId();
        
        // Compare transform information with optimized checks
        ActorState.Transform prevTransform = previousState.getTransform();
        ActorState.Transform currTransform = currentState.getTransform();
        
        if (prevTransform != null && currTransform != null) {
            // Check location changes with tolerance for floating point precision
            List<Double> prevLoc = prevTransform.getLocation();
            List<Double> currLoc = currTransform.getLocation();
            if (prevLoc != null && currLoc != null) {
                if (!isLocationEqual(prevLoc, currLoc)) {
                    log.debug("Actor {} location changed: {} -> {}", actorId, prevLoc, currLoc);
                    return true;
                }
            } else if (!Objects.equals(prevLoc, currLoc)) {
                log.debug("Actor {} location changed (null check): {} -> {}", actorId, prevLoc, currLoc);
                return true;
            }
            
            // Check rotation changes
            List<Double> prevRot = prevTransform.getRotation();
            List<Double> currRot = currTransform.getRotation();
            if (!Objects.equals(prevRot, currRot)) {
                log.debug("Actor {} rotation changed: {} -> {}", actorId, prevRot, currRot);
                return true;
            }
        } else if (prevTransform != currTransform) {
            log.debug("Actor {} transform changed (null check)", actorId);
            return true;
        }
        
        // Compare velocity information with tolerance
        ActorState.Velocity prevVelocity = previousState.getVelocity();
        ActorState.Velocity currVelocity = currentState.getVelocity();
        
        if (prevVelocity != null && currVelocity != null) {
            List<Double> prevLinear = prevVelocity.getLinear();
            List<Double> currLinear = currVelocity.getLinear();
            
            // Compare linear velocities with tolerance
            if (prevLinear != null && currLinear != null) {
                if (!isVelocityEqual(prevLinear, currLinear)) {
                    log.debug("Actor {} velocity changed: {} -> {}", actorId, prevLinear, currLinear);
                    return true;
                }
            } else if (!Objects.equals(prevLinear, currLinear)) {
                log.debug("Actor {} velocity changed (null check): {} -> {}", actorId, prevLinear, currLinear);
                return true;
            }
        } else if (prevVelocity != currVelocity) {
            log.debug("Actor {} velocity changed (null check)", actorId);
            return true;
        }
        
        // Detailed debug for why no change was detected if we suspect it should have
        if (log.isTraceEnabled()) {
             log.trace("No change detected for actor {}. Loc: {}, Vel: {}", 
                     actorId, 
                     currTransform != null ? currTransform.getLocation() : "null",
                     currVelocity != null ? currVelocity.getLinear() : "null");
        }
        
        return false;
    }
    
    /**
     * Check if two location lists are equal within tolerance
     */
    private boolean isLocationEqual(List<Double> loc1, List<Double> loc2) {
        if (loc1 == null || loc2 == null) {
            return loc1 == loc2;
        }
        if (loc1.size() != loc2.size()) return false;
        
        final double TOLERANCE = LOCATION_TOLERANCE;
        for (int i = 0; i < loc1.size(); i++) {
            Double val1 = loc1.get(i);
            Double val2 = loc2.get(i);
            if (val1 == null || val2 == null) {
                if (val1 != val2) {
                    return false;
                }
            } else if (Math.abs(val1 - val2) > TOLERANCE) {
                log.debug("Location change detected at index {}: {} vs {} (diff: {})", i, val1, val2, Math.abs(val1 - val2));
                return false;
            }
        }
        return true;
    }
    
    /**
     * Check if two velocity lists are equal within tolerance
     */
    private boolean isVelocityEqual(List<Double> vel1, List<Double> vel2) {
        if (vel1 == null || vel2 == null) {
            return vel1 == vel2;
        }
        if (vel1.size() != vel2.size()) return false;
        
        final double TOLERANCE = VELOCITY_TOLERANCE;
        for (int i = 0; i < vel1.size(); i++) {
            Double val1 = vel1.get(i);
            Double val2 = vel2.get(i);
            if (val1 == null || val2 == null) {
                if (val1 != val2) {
                    return false;
                }
            } else if (Math.abs(val1 - val2) > TOLERANCE) {
                log.debug("Velocity change detected at index {}: {} vs {} (diff: {})", i, val1, val2, Math.abs(val1 - val2));
                return false;
            }
        }
        return true;
    }

    /**
     * Clear all cached states (useful for resetting change detection)
     */
    public void clearStateCache() {
        previousActorStates.clear();
        previousTrafficLightStates.clear();
        log.debug("Cleared state cache");
    }

    /**
     * Get current cached actor states
     * @return Map of actor ID to actor state
     */
    public Map<String, ActorState> getCachedActorStates() {
        return new HashMap<>(previousActorStates);
    }

    /**
     * Get current cached traffic light states
     * @return Map of traffic light ID to traffic light state
     */
    public Map<String, Map<String, Object>> getCachedTrafficLightStates() {
        return new HashMap<>(previousTrafficLightStates);
    }
    
    /**
     * Clean up resources and close connections
     * This method should be called when the client is no longer needed
     */
    public void cleanup() {
        synchronized (connectionLock) {
            if (isConnected) {
                try {
                    disconnect();
                } catch (Exception e) {
                    log.warn("Error during cleanup disconnect: {}", e.getMessage());
                }
            }
        }
        
        // Clear caches to free memory
        clearStateCache();
        
        // Reset request counter
        requestCounter.set(0);
        
        log.debug("CARLA XML-RPC client cleanup completed");
    }
    
    /**
     * Get connection statistics for monitoring
     * @return Map containing connection statistics
     */
    public Map<String, Object> getConnectionStats() {
        Map<String, Object> stats = new HashMap<>();
        stats.put("isConnected", isConnected);
        stats.put("serverType", serverType);
        stats.put("serverUrl", serverUrl.toString());
        stats.put("requestCount", requestCounter.get());
        stats.put("cachedActorStates", previousActorStates.size());
        stats.put("cachedTrafficLightStates", previousTrafficLightStates.size());
        return stats;
    }
}
