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

import org.eclipse.mosaic.lib.CommonUtil.CommonInstanceManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManager extends CommonInstanceManager<CarmaMessengerInstance, CarmaMessengerRegistrationMessage>{


    // TODO: Verify actual port for CARMA Platform NS-3 adapter
    
    private static final int BRIDGE_TARGET_PORT = 5500;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onNewRegistration(CarmaMessengerRegistrationMessage registration) {
        super.setTargetPort(5600);
        if (!managedInstances.containsKey(registration.getVehicleRole())) {
            try {
                newCarmaMessengerInstance(
                    registration.getVehicleId(),
                    registration.getVehicleRole(),
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
            log.warn("Received duplicate registration for vehicle " + registration.getVehicleRole());
        }
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
                    currentInstance.getVehicleId(), 
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
}
