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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonInstanceManager;
import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManager extends CommonInstanceManager<CarmaMessengerInstance, CarmaMessengerRegistrationMessage>{

    private static final int BRIDGE_TARGET_PORT = 5500;
    private final Logger log = LoggerFactory.getLogger(this.getClass());
    private final Map<String, Object> registrationList = new HashMap<>();

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onNewRegistration(Object registration) {
        super.setTargetPort(5600);
        // Check registration type and get vehicle role
        String vehicleRole = null;
        if (registration instanceof CarmaMessengerRegistrationMessage) {
            vehicleRole = ((CarmaMessengerRegistrationMessage) registration).getVehicleRole();
        } else if (registration instanceof CarmaMessengerBridgeRegistrationMessage) {
            vehicleRole = ((CarmaMessengerBridgeRegistrationMessage) registration).getVehicleRole();
        }

        if (vehicleRole == null) {
            throw new IllegalArgumentException("Invalid registration type.");
        }

        if (registrationList == null || !registrationList.containsKey(vehicleRole)) {
            registrationList.put(vehicleRole, registration);
        } else {
            if (!managedInstances.containsKey(vehicleRole)) {
                try {
                    // Determine necessary values for creating a new instance
                    CarmaMessengerRegistrationMessage regMessage = null;
                    CarmaMessengerBridgeRegistrationMessage bridgeMessage = null;
                    InetAddress rxMessageIp = null;
                    int rxTimeSyncPort = 0;

                    if (registration instanceof CarmaMessengerRegistrationMessage) {
                        regMessage = (CarmaMessengerRegistrationMessage) registration;
                        bridgeMessage = (CarmaMessengerBridgeRegistrationMessage) registrationList.get(vehicleRole);
                        rxMessageIp = InetAddress.getByName(regMessage.getRxMessageIpAddress());
                        rxTimeSyncPort = regMessage.getRxTimeSyncPort();
                    } else if (registration instanceof CarmaMessengerBridgeRegistrationMessage) {
                        bridgeMessage = (CarmaMessengerBridgeRegistrationMessage) registration;
                        regMessage = (CarmaMessengerRegistrationMessage) registrationList.get(vehicleRole);
                        rxMessageIp = InetAddress.getByName(bridgeMessage.getRxMessageIpAddress());
                        rxTimeSyncPort = bridgeMessage.getRxTimeSyncPort();
                    }

                    // Initialize parameters for new instance
                    String vehicleId = regMessage.getVehicleId();              
                    int rxMessagePort = regMessage.getRxMessagePort();                    
                    int rxBridgeMessagePort = regMessage.getRxBridgeMessagePort();
                    int rxVehicleStatusPort = bridgeMessage.getRxVehicleStatusPort();
                    int rxTrafficEventPort = bridgeMessage.getRxTrafficEventPort();

                    // TODO: update 0 values as needed
                    newCarmaMessengerInstance(
                        vehicleId, vehicleRole, rxMessageIp, rxMessagePort, 
                        rxTimeSyncPort, rxBridgeMessagePort,
                        rxVehicleStatusPort, rxTrafficEventPort
                    );

                } catch (Exception e) {
                    throw new RuntimeException("Failed to create CarmaMessenger instance", e);
                }
            } else {
                log.warn("Received duplicate registration for vehicle " + vehicleRole);
            }
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
    private void newCarmaMessengerInstance(String carmaMessengerVehId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, int rxBridgeMessagePort, int rxVehicleStatusPort, int rxTrafficEventPort) {
        CarmaMessengerInstance tmp = new CarmaMessengerInstance(carmaMessengerVehId, sumoRoleName, targetAddress, v2xPort, timeSyncPort, rxBridgeMessagePort,rxVehicleStatusPort, rxTrafficEventPort);
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

    public List<String> getVehicleIds(){

        List<String> result = new ArrayList<>();

        for(CarmaMessengerInstance name : managedInstances.values()){
            result.add(name.getVehicleId());
        }
            
        return result;
        
    }

    public void onDetectedTrafficEvents(MsgerTrafficEvent message) throws IOException {
        if (managedInstances.size() == 0) {
            log.debug("There are no registered instances");
        }
        else {
            Gson gson = new Gson();
            byte[] bytes = gson.toJson(message).getBytes();
            for (CarmaMessengerInstance currentInstance : managedInstances.values()) {
                if(currentInstance.getRoleName().equals(message.getVehicleId())){
                    currentInstance.sendTrafficEventMsgs(bytes);
                    log.debug(message.toString());
                }
            }
        }
    }
}
