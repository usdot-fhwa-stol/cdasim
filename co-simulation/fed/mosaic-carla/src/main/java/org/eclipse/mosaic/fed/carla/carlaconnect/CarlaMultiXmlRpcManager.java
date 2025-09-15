/*
 * Copyright (c) 2025 MSC Lab, University of Georgia. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */

package org.eclipse.mosaic.fed.carla.carlaconnect;

import org.apache.xmlrpc.XmlRpcException;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Multi-connection manager for CARLA XML-RPC clients
 * 
 * This manager handles multiple XML-RPC connections to different CARLA servers,
 * providing a unified interface for sensor and actor operations.
 */
public class CarlaMultiXmlRpcManager {

    private static final Logger log = LoggerFactory.getLogger(CarlaMultiXmlRpcManager.class);
    
    private final Map<CarlaXmlRpcClient.ServerType, CarlaXmlRpcClient> clients = new ConcurrentHashMap<>();
    private final Map<CarlaXmlRpcClient.ServerType, Boolean> connectionStatus = new ConcurrentHashMap<>();
    
    /**
     * Add a client for a specific server type
     * @param serverType Type of server (SENSOR_LIB or ACTOR_LIB)
     * @param serverUrl URL of the server
     * @throws MalformedURLException if URL is invalid
     */
    public void addClient(CarlaXmlRpcClient.ServerType serverType, String serverUrl) throws MalformedURLException {
        URL url = new URL(serverUrl);
        CarlaXmlRpcClient client = new CarlaXmlRpcClient(url, serverType);
        clients.put(serverType, client);
        connectionStatus.put(serverType, false);
        log.info("Added {} client for URL: {}", serverType, serverUrl);
    }
    
    /**
     * Connect to all registered servers
     * @param retryAttempts Number of retry attempts for each connection
     * @throws XmlRpcException if any connection fails
     * @throws InterruptedException if interrupted during connection
     */
    public void connectAll(int retryAttempts) throws XmlRpcException, InterruptedException {
        for (Map.Entry<CarlaXmlRpcClient.ServerType, CarlaXmlRpcClient> entry : clients.entrySet()) {
            CarlaXmlRpcClient.ServerType serverType = entry.getKey();
            CarlaXmlRpcClient client = entry.getValue();
            
            try {
                log.info("Connecting to {} server...", serverType);
                client.connect(retryAttempts);
                connectionStatus.put(serverType, true);
                log.info("Successfully connected to {} server", serverType);
            } catch (XmlRpcException e) {
                log.error("Failed to connect to {} server: {}", serverType, e.getMessage());
                connectionStatus.put(serverType, false);
                throw e;
            }
        }
    }
    
    /**
     * Disconnect from all servers
     */
    public void disconnectAll() {
        for (Map.Entry<CarlaXmlRpcClient.ServerType, CarlaXmlRpcClient> entry : clients.entrySet()) {
            CarlaXmlRpcClient.ServerType serverType = entry.getKey();
            CarlaXmlRpcClient client = entry.getValue();
            
            try {
                client.disconnect();
                connectionStatus.put(serverType, false);
                log.info("Disconnected from {} server", serverType);
            } catch (Exception e) {
                log.warn("Error disconnecting from {} server: {}", serverType, e.getMessage());
            }
        }
    }
    
    /**
     * Check if a specific server type is connected
     * @param serverType Type of server to check
     * @return true if connected
     */
    public boolean isConnected(CarlaXmlRpcClient.ServerType serverType) {
        CarlaXmlRpcClient client = clients.get(serverType);
        if (client == null) {
            return false;
        }
        
        boolean connected = client.isConnected();
        connectionStatus.put(serverType, connected);
        return connected;
    }
    
    /**
     * Check if all servers are connected
     * @return true if all registered servers are connected
     */
    public boolean areAllConnected() {
        for (CarlaXmlRpcClient.ServerType serverType : clients.keySet()) {
            if (!isConnected(serverType)) {
                return false;
            }
        }
        return !clients.isEmpty();
    }
    
    /**
     * Get client for specific server type
     * @param serverType Type of server
     * @return CarlaXmlRpcClient instance or null if not found
     */
    public CarlaXmlRpcClient getClient(CarlaXmlRpcClient.ServerType serverType) {
        return clients.get(serverType);
    }
    
    // ========== Sensor Operations (SENSOR_LIB server) ==========
    
    /**
     * Create sensor using SENSOR_LIB server
     * @param registration DetectorRegistration interaction
     * @throws XmlRpcException if creation fails
     */
    public void createSensor(DetectorRegistration registration) throws XmlRpcException {
        CarlaXmlRpcClient sensorClient = getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
        if (sensorClient == null) {
            throw new XmlRpcException("SENSOR_LIB client not available");
        }
        sensorClient.createSensor(registration);
    }
    
    /**
     * Get detected objects from SENSOR_LIB server
     * @param infrastructureId Infrastructure ID
     * @param sensorId Sensor ID
     * @return Array of detected objects
     * @throws XmlRpcException if retrieval fails
     */
    public DetectedObject[] getDetectedObjects(String infrastructureId, String sensorId) throws XmlRpcException {
        CarlaXmlRpcClient sensorClient = getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
        if (sensorClient == null) {
            throw new XmlRpcException("SENSOR_LIB client not available");
        }
        return sensorClient.getDetectedObjects(infrastructureId, sensorId);
    }
    
    // ========== Actor Operations (ACTOR_LIB server) ==========
    
    /**
     * Advance simulation using ACTOR_LIB server
     * @return true if successful
     */
    public boolean advanceSimulation() {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for advanceSimulation");
            return false;
        }
        return actorClient.advanceSimulation();
    }
    
    /**
     * Get all actors from ACTOR_LIB server
     * @return Map of actor information
     */
    public Map<String, Map<String, Object>> getAllActors() {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for getAllActors");
            return new HashMap<>();
        }
        return actorClient.getAllActors();
    }
    
    /**
     * Get all traffic light states from ACTOR_LIB server
     * @return List of traffic light states
     */
    public List<Map<String, Object>> getAllTrafficLightStates() {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for getAllTrafficLightStates");
            return new ArrayList<>();
        }
        return actorClient.getAllTrafficLightStates();
    }
    
    /**
     * Spawn actor using ACTOR_LIB server
     * @param actorType Type of actor
     * @param actorId Actor ID
     * @param location Location [x, y, z]
     * @param rotation Rotation [pitch, yaw, roll]
     * @param properties Additional properties
     * @return true if successful
     */
    public boolean spawnActor(String actorType, String actorId, List<Double> location, 
                             List<Double> rotation, Map<String, Object> properties) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for spawnActor");
            return false;
        }
        return actorClient.spawnActor(actorType, actorId, location, rotation, properties);
    }
    
    /**
     * Destroy actor using ACTOR_LIB server
     * @param actorId Actor ID
     * @return true if successful
     */
    public boolean destroyActor(String actorId) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for destroyActor");
            return false;
        }
        return actorClient.destroyActor(actorId);
    }
    
    /**
     * Update actor transform using ACTOR_LIB server
     * @param actorId Actor ID
     * @param location New location
     * @param rotation New rotation
     * @return true if successful
     */
    public boolean updateActorTransform(String actorId, List<Double> location, List<Double> rotation) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for updateActorTransform");
            return false;
        }
        return actorClient.updateActorTransform(actorId, location, rotation);
    }
    
    /**
     * Update actor velocity using ACTOR_LIB server
     * @param actorId Actor ID
     * @param velocity New velocity
     * @return true if successful
     */
    public boolean updateActorVelocity(String actorId, List<Double> velocity) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for updateActorVelocity");
            return false;
        }
        return actorClient.updateActorVelocity(actorId, velocity);
    }
    
    /**
     * Set actor state properties using ACTOR_LIB server
     * @param actorId Actor ID
     * @param properties Properties to set
     * @return true if successful
     */
    public boolean setActorStateProperties(String actorId, Map<String, Object> properties) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for setActorStateProperties");
            return false;
        }
        return actorClient.setActorStateProperties(actorId, properties);
    }
    
    /**
     * Set traffic light state using ACTOR_LIB server
     * @param trafficLightId Traffic light ID
     * @param state New state
     * @return true if successful
     */
    public boolean setTrafficLightState(String trafficLightId, String state) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for setTrafficLightState");
            return false;
        }
        return actorClient.setTrafficLightState(trafficLightId, state);
    }
    
    /**
     * Set traffic light timer using ACTOR_LIB server
     * @param trafficLightId Traffic light ID
     * @param timeSeconds Time in seconds
     * @return true if successful
     */
    public boolean setTrafficLightTimer(String trafficLightId, double timeSeconds) {
        CarlaXmlRpcClient actorClient = getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        if (actorClient == null) {
            log.warn("ACTOR_LIB client not available for setTrafficLightTimer");
            return false;
        }
        return actorClient.setTrafficLightTimer(trafficLightId, timeSeconds);
    }
    
    /**
     * Get connection status for all servers
     * @return Map of server types to connection status
     */
    public Map<CarlaXmlRpcClient.ServerType, Boolean> getConnectionStatus() {
        Map<CarlaXmlRpcClient.ServerType, Boolean> status = new HashMap<>();
        for (CarlaXmlRpcClient.ServerType serverType : clients.keySet()) {
            status.put(serverType, isConnected(serverType));
        }
        return status;
    }
}
