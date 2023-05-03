/*
 * Copyright (C) 2019-2022 LEIDOS.
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

package org.eclipse.mosaic.fed.carma.ambassador;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoCircle;
import org.eclipse.mosaic.lib.objects.addressing.AdHocMessageRoutingBuilder;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

/**
 * Session management class for CARMA Platform instances communicating with MOSAIC
 */
public class CarmaInstanceManager {
    private Map<String, CarmaInstance>  managedInstances = new HashMap<>();
    private double currentSimulationTime;

    // TODO: Verify actual port for CARMA Platform NS-3 adapter
    private static final int TARGET_PORT = 5374;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onNewRegistration(CarmaRegistrationMessage registration) {
        if (!managedInstances.containsKey(registration.getCarlaVehicleRole())) {
            try {
                newCarmaInstance(
                    registration.getCarmaVehicleId(),
                    registration.getCarlaVehicleRole(),
                    InetAddress.getByName(registration.getRxMessageIpAddress()),
                    registration.getRxMessagePort()
                );
            } catch (UnknownHostException e) {
                throw new RuntimeException(e);
            }
        } else {
            // log warning
        }
    }
    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param sourceAddr The V2X Message received
     * @param txMsg The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public V2xMessageTransmission onV2XMessageTx(InetAddress sourceAddr, CarmaV2xMessage txMsg) {
        CarmaInstance sender = null;
        for (CarmaInstance ci : managedInstances.values()) {
            if (ci.getTargetAddress().equals(sourceAddr)) {
                sender = ci;
                break;
            }
            log.info("Instance {} with target address {} can not match with source address {}", ci.getCarlaRoleName(), ci.getTargetAddress(), sourceAddr.toString());
        }

        if (sender == null) {
            // Unregistered instance attempting to send messages
            throw new IllegalStateException("Unregistered CARMA Platform instance attempting to send messages via MOSAIC");
        }

        AdHocMessageRoutingBuilder messageRoutingBuilder = new AdHocMessageRoutingBuilder(
                sender.getCarlaRoleName(), sender.getLocation()).viaChannel(AdHocChannel.SCH4);

        // TODO: Get maximum broadcast radius from configuration file.
        MessageRouting routing = messageRoutingBuilder.geoBroadCast(new GeoCircle(sender.getLocation(), 300));

        log.info("Preparing to generate V2XMessageTransmission interaction for transmission on MOSAIC event bus...");
        log.info("sim time: " + currentSimulationTime);
        log.info("sender id: " + sender.getCarlaRoleName());
        log.info("location: " + sender.getLocation().toString());
        log.info("txMsg non-null? " + (txMsg != null));
        log.info("payload: " + txMsg.getPayload());
        return new V2xMessageTransmission((long) currentSimulationTime, new ExternalV2xMessage(routing,
                new ExternalV2xContent((long) currentSimulationTime, sender.getLocation(), txMsg.getPayload())));
    }

    /**
     * Callback to be invoked when the simulation emits a VehicleUpdates event, used to track the location of CARMA
     * Platform instances in this manager.
     * @param vui The vehicle update information
     */
    public void onVehicleUpdates(VehicleUpdates vui) {
        for (VehicleData veh : vui.getUpdated()) {
            if (managedInstances.containsKey(veh.getName())) {
                managedInstances.get(veh.getName()).setLocation(veh.getPosition());
            }
        }
    }

    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param rxMsg The V2X Message received
     * @param rxVehicleId The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public void onV2XMessageRx(byte[] rxMsg, String rxVehicleId) {
        if (!managedInstances.containsKey(rxVehicleId))  {
            return;
        }

        CarmaInstance carma = managedInstances.get(rxVehicleId);
        try {
            carma.sendMsgs(rxMsg);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Helper function to configure a new CARMA Platform instance object upon registration
     * @param carmaVehId The CARMA Platform vehicle ID (e.g. it's license plate number)
     * @param carlaRoleName The Role Name associated with the CARMA Platform's ego vehicle in CARLA
     * @param targetAddress The IP address to which received simulated V2X messages should be sent
     * @param targetPort The port to which received simulated V2X messages should be sent
     */
    private void newCarmaInstance(String carmaVehId, String carlaRoleName, InetAddress targetAddress, int targetPort) {
        CarmaInstance tmp = new CarmaInstance(carmaVehId, carlaRoleName, targetAddress, targetPort);
        try {
            tmp.bind();
            log.info("New CARMA instance '{}' registered with CARMA Instance Manager.", carlaRoleName);
        } catch (IOException e) {
            log.error("Failed to bind CARMA instance with ID '{}' to its RX message socket: {}",
            carlaRoleName, e.getMessage());
            log.error("Stack trace:", e);
            throw new RuntimeException(e);
        }
        managedInstances.put(carlaRoleName, tmp);
    }

    /**
     * External helper function to allow the ambassador to check if a given vehicle ID is a registered CARMA Platform
     * instance
     * @param mosiacVehicleId The id to check
     * @return True if managed by this object (e.g., is a registered CARMA Platform vehicle). false o.w.
     */
    public boolean checkIfRegistered(String mosiacVehicleId) {
        return managedInstances.keySet().contains(mosiacVehicleId);
    }
}
