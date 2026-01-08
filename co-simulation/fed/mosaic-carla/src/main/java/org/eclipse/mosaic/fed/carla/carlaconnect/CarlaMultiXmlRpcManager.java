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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.xmlrpc.XmlRpcException;

/**
 * Multi-connection manager for CARLA XML-RPC clients
 * 
 * This manager handles multiple XML-RPC connections to different CARLA servers,
 * providing connection management and client retrieval functionality.
 * CARLA operations are accessed directly through the returned client instances.
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
     * Attempts to connect to each server independently, allowing partial success.
     * @param retryAttempts Number of retry attempts for each connection
     * @throws InterruptedException if interrupted during connection
     */
    public void connectAll(int retryAttempts) throws InterruptedException {
        int successCount = 0;
        int failureCount = 0;
        
        for (Map.Entry<CarlaXmlRpcClient.ServerType, CarlaXmlRpcClient> entry : clients.entrySet()) {
            CarlaXmlRpcClient.ServerType serverType = entry.getKey();
            CarlaXmlRpcClient client = entry.getValue();
            
            try {
                log.info("Connecting to {} server...", serverType);
                client.connect(retryAttempts);
                connectionStatus.put(serverType, true);
                successCount++;
                log.info("Successfully connected to {} server", serverType);
            } catch (XmlRpcException e) {
                log.error("Failed to connect to {} server: {}", serverType, e.getMessage());
                connectionStatus.put(serverType, false);
                failureCount++;
                // Continue trying other servers instead of throwing exception
            } catch (Exception e) {
                log.error("Unexpected error connecting to {} server: {}", serverType, e.getMessage());
                connectionStatus.put(serverType, false);
                failureCount++;
            }
        }
        
        // Log summary of connection results
        if (successCount > 0 && failureCount > 0) {
            log.warn("Partial connection success: {} connected, {} failed", successCount, failureCount);
        } else if (failureCount > 0) {
            log.error("All connection attempts failed ({} failures)", failureCount);
        } else if (successCount > 0) {
            log.info("All servers connected successfully ({} connections)", successCount);
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
