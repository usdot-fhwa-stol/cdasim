/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.eclipse.mosaic.fed.infrastructure.ambassador;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.fed.infrastructure.ambassador.InfrastructureRegistrationMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Session management class for Infrastructure instances communicating with
 * MOSAIC.
 * 
 * This class is responsible for managing instances of infrastructure registered
 * with the MOSAIC system. It provides methods for registering new instances,
 * checking if instances are registered, and storing and retrieving instances
 * from a map.
 */
public class InfrastructureInstanceManager {
    private Map<String, InfrastructureInstance> managedInstances = new HashMap<>();
    private double currentSimulationTime;
    private static final Logger log = LoggerFactory.getLogger(InfrastructureInstanceManager.class);

    /**
     * Register a new infrastructure instance with the MOSAIC system.
     * 
     * This method takes an InfrastructureRegistrationMessage, converts it to an
     * InfrastructureInstance, and adds it to the managedInstances map if it is not
     * already present.
     * 
     * @param registration The InfrastructureRegistrationMessage to be registered.
     * 
     */
    public void onNewRegistration(InfrastructureRegistrationMessage registration) {
        if (!managedInstances.containsKey(registration.getInfrastructureId())) {
            try {
                newInfrastructureInstance(
                        registration.getInfrastructureId(),
                        InetAddress.getByName(registration.getRxMessageIpAddress()),
                        registration.getRxMessagePort(),
                        registration.getTimeSyncPort(),
                        registration.getLocation());
            } catch (UnknownHostException e) {
                log.error("Failed to create infrastructure instance with ID '{}' due to an unknown host exception: {}",
                        registration.getInfrastructureId(), e.getMessage());
                log.error("Stack trace:", e);
            }
        } else {
            log.warn("Registration message received for already registered infrastructure with ID: {}",
                    registration.getInfrastructureId());
        }
    }

    /**
     * Create a new InfrastructureInstance and add it to the managedInstances map.
     * 
     * This method creates a new InfrastructureInstance with the provided parameters
     * and adds it to the managedInstances map.
     * 
     * @param infrastructureId   The ID of the new instance.
     * @param rxMessageIpAddress The IP address to receive messages on.
     * @param rxMessagePort      The port to receive messages on.
     * @param timeSyncPort       The port for time synchronization.
     * @param location           The location of the instance.
     * 
     */
    private void newInfrastructureInstance(String infrastructureId, InetAddress rxMessageIpAddress, int rxMessagePort,
            int timeSyncPort, GeoPoint location) {
        InfrastructureInstance tmp = new InfrastructureInstance(infrastructureId, rxMessageIpAddress, rxMessagePort,
                timeSyncPort, location);
        try {
            tmp.bind();
        } catch (IOException e) {
            log.error("Failed to bind infrastructure instance with ID '{}' to its RX message socket: {}",
                    infrastructureId, e.getMessage());
            log.error("Stack trace:", e);
        }
        managedInstances.put(infrastructureId, tmp);
    }

    /**
     * External helper function to allow the ambassador to check if a given vehicle
     * ID is a registered CARMA Platform instance
     * 
     * @param infrastructureId The id to check
     * @return True if managed by this object (e.g., is a registered CARMA Platform
     *         vehicle). false o.w.
     */
    public boolean checkIfRegistered(String infrastructureId) {
        return managedInstances.keySet().contains(infrastructureId);

    public Map<String, InfrastructureInstance> getManagedInstances() {
        return managedInstances;
    }
}
