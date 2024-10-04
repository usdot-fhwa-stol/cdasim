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
package org.eclipse.mosaic.fed.carmamessenger.ambassador;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.fed.carma.ambassador.CarmaInstanceManager;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoCircle;
import org.eclipse.mosaic.lib.objects.addressing.AdHocMessageRoutingBuilder;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManager extends CarmaInstanceManager{
    private Map<String, CarmaMessengerInstance>managedInstances = new HashMap<>();

    // TODO: Verify actual port for CARMA Platform NS-3 adapter
    private static final int ADAPTER_TARGET_PORT = 5500;
    private static final int BRIDGE_TARGET_PORT = 5500;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onNewRegistration(CarmaMessengerRegistrationMessage registration) {
        if (!managedInstances.containsKey(registration.getSumoVehicleRole())) {
            try {
                newCarmaMessengerInstance(
                    registration.getCarmaVehicleId(),
                    registration.getSumoVehicleRole(),
                    InetAddress.getByName(registration.getRxMessageIpAddress()),
                    registration.getRxMessagePort(),
                    registration.getRxTimeSyncPort(),
                    registration.getMessengerEmergencyState(),
                    registration.getRxBridgeMessagePort()
                );
            } catch (UnknownHostException e) {
                throw new RuntimeException(e);
            }
        } else {
            // log warning
            log.warn("Received duplicate registration for vehicle " + registration.getSumoVehicleRole());
        }
    }
    /**
     * Method to be invoked when CARMA Ambassador receives a V2X message from CARMA Platform. Creates
     * V2xMessageTransmission interaction to be sent on the MOSIAC RTI.
     * @param sourceAddr the ip address of the CARMA Platform instance that sent the message.
     * @param txMsg The V2X Message received.
     * @param time The timestamp at which the interaction occurs.
     * @throws IllegalStateException if sourceAddr does not match any address in the managed instances.
     */
    @Override
    public V2xMessageTransmission onV2XMessageTx(InetAddress sourceAddr, CarmaV2xMessage txMsg, long time) {
        CarmaMessengerInstance sender = null;
        // Find the CarmaInstance with sourceAddr.
        for (CarmaMessengerInstance ci : managedInstances.values()) {
            if (ci.getTargetAddress().equals(sourceAddr)) {
                sender = ci;
                break;
            }
        }
        // Unregistered instance attempting to send messages
        if (sender == null) {
            throw new IllegalStateException("Unregistered CARMA Platform instance attempting to send messages via MOSAIC");
        }
        AdHocMessageRoutingBuilder messageRoutingBuilder = new AdHocMessageRoutingBuilder(
                sender.getSumoRoleName(), sender.getLocation()).viaChannel(AdHocChannel.CCH);
        // TODO: Get maximum broadcast radius from configuration file.
        MessageRouting routing = messageRoutingBuilder.geoBroadCast(new GeoCircle(sender.getLocation(), 300));
        log.debug("Generating V2XMessageTransmission interaction sim time: {}, sender id: {}, location: {}, type: {}, payload: {}", 
                time, 
                sender.getCarmaMessengerVehicleId(),
                sender.getLocation(),
                txMsg.getType(),
                txMsg.getPayload()
            );
        return new V2xMessageTransmission( time, new ExternalV2xMessage(routing,
                new ExternalV2xContent( time, sender.getLocation(), txMsg.getPayload())));
    }

    /**
     * This function is used to send out encoded timestep update to all registered
     * instances the manager has on the managed instances map
     * 
     * @param message This time message is used to store current seq and timestep
     *                from the ambassador side
     * @throws IOException
     */
    @Override
    public void onTimeStepUpdate(TimeSyncMessage message) throws IOException {
        if (managedInstances.size() == 0) {
            log.debug("There are no registered instances");
        }
        else {
            Gson gson = new Gson();
            byte[] bytes = gson.toJson(message).getBytes();
            for (CarmaMessengerInstance currentInstance : managedInstances.values()) {
                log.debug("Sending CARMA Messenger instance {} at {}:{} time sync message for time {}!" ,
                    currentInstance.getCarmaMessengerVehicleId(), 
                    currentInstance.getTargetAddress(), 
                    currentInstance.getTimeSyncPort(),
                    currentInstance.getMessengerEmergencyState(), 
                    message.getTimestep());
                currentInstance.sendTimeSyncMsg(bytes);
            }
        }
    }


    /**
     * Helper function to configure a new CARMA Platform instance object upon registration
     * @param carmaMessengerVehId The CARMA Platform vehicle ID (e.g. it's license plate number)
     * @param sumoRoleName The Role Name associated with the CARMA Platform's ego vehicle in CARLA
     * @param targetAddress The IP address to which received simulated V2X messages should be sent
     * @param v2xPort The port to which received simulated V2X messages should be sent
     * @param timeSyncPort The port to which to send time sync messages.
     */
    private void newCarmaMessengerInstance(String carmaMessengerVehId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, String messengerEmergencyState, int rxBridgeMessagePort) {
        CarmaMessengerInstance tmp = new CarmaMessengerInstance(carmaMessengerVehId, sumoRoleName, targetAddress, v2xPort, timeSyncPort, messengerEmergencyState, rxBridgeMessagePort);
        try {
            tmp.bind();
            log.info("New CARMA Messenger instance '{}' registered with CARMA Instance Manager.", sumoRoleName);
        } catch (IOException e) {
            log.error("Failed to bind CARMA Messenger instance with ID '{}' to its RX message socket: {}",
            sumoRoleName, e.getMessage());
            log.error("Stack trace:", e);
            throw new RuntimeException(e);
        }
        managedInstances.put(sumoRoleName, tmp);
    }
    /**
     * External helper function to allow the ambassador to check if a given vehicle ID is a registered CARMA Platform
     * instance
     * @param mosiacVehicleId The id to check
     * @return True if managed by this object (e.g., is a registered CARMA Platform vehicle). false o.w.
     */
    @Override
    public boolean checkIfRegistered(String mosiacVehicleId) {
        return this.managedInstances.keySet().contains(mosiacVehicleId);
    }

    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param rxMsg The V2X Message received
     * @param rxVehicleId The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    @Override
    public void onV2XMessageRx(byte[] rxMsg, String rxVehicleId) {
        if (!managedInstances.containsKey(rxVehicleId))  {
            return;
        }

        CarmaMessengerInstance carmaMessenger = managedInstances.get(rxVehicleId);
        try {
            carmaMessenger.sendV2xMsgs(rxMsg);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
