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

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoCircle;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.addressing.AdHocMessageRoutingBuilder;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

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

    private final Logger log = LoggerFactory.getLogger(this.getClass());
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
            int timeSyncPort, CartesianPoint location) {
        InfrastructureInstance tmp = new InfrastructureInstance(infrastructureId, rxMessageIpAddress, rxMessagePort,
                timeSyncPort, location);
        try {
            tmp.bind();
            log.info("New Infrastructure instance '{}' registered with Infrastructure Instance Manager.", infrastructureId);
        } catch (IOException e) {
            log.error("Failed to bind infrastructure instance with ID '{}' to its RX message socket: {}",
                    infrastructureId, e.getMessage());
            log.error("Stack trace:", e);
        }
        managedInstances.put(infrastructureId, tmp);
    }

    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param sourceAddr The V2X Message received
     * @param txMsg The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public V2xMessageTransmission onV2XMessageTx(InetAddress sourceAddr, CarmaV2xMessage txMsg, long time) {
        InfrastructureInstance sender = null;
        for (InfrastructureInstance ci : managedInstances.values()) {
            if (ci.getTargetAddress().equals(sourceAddr)) {
                sender = ci;
            }
        }

        if (sender == null) {
            // Unregistered instance attempting to send messages
            throw new IllegalStateException("Unregistered CARMA Streets/V2XHub instance attempting to send messages via MOSAIC");
        }

        AdHocMessageRoutingBuilder messageRoutingBuilder = new AdHocMessageRoutingBuilder(
                sender.getInfrastructureId(), sender.getLocation().toGeo()).viaChannel(AdHocChannel.CCH);

        // TODO: Get maximum broadcast radius from configuration file.
        MessageRouting routing = messageRoutingBuilder.geoBroadCast(new GeoCircle(sender.getLocation().toGeo(), 300));

        return new V2xMessageTransmission(time, new ExternalV2xMessage(routing,
                new ExternalV2xContent(time, sender.getLocation().toGeo(), txMsg.getPayload())));
    }

    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param rxMsg The V2X Message received
     * @param rxRsuId The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public void onV2XMessageRx(byte[] rxMsg, String rxRsuId) {
        if (!managedInstances.containsKey(rxRsuId))  {
            return;
        }

        InfrastructureInstance rsu = managedInstances.get(rxRsuId);
        try {
            rsu.sendMsgs(rxMsg);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
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
    }
    public Map<String, InfrastructureInstance> getManagedInstances() {
        return managedInstances;
    }
}
