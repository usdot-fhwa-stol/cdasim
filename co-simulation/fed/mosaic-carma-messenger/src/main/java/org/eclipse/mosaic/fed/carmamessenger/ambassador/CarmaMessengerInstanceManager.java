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

import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonInstanceManager;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;
import org.eclipse.mosaic.lib.objects.vehicle.MsgerVehicleStatus;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManager extends CommonInstanceManager<CarmaMessengerInstance, CarmaMessengerRegistrationMessage>{

    private static final int BRIDGE_TARGET_PORT = 5500;
    private final Logger log = LoggerFactory.getLogger(this.getClass());
    private final Map<String, CarmaMessengerRegistrationMessage> ns3RegistrationList = new HashMap<>();
    private final Map<String, CarmaMessengerBridgeRegistrationMessage> bridgeRegistrationList = new HashMap<>();

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onMsgerNewRegistration(Object registration) {
        super.setTargetPort(5600);
        // Check registration type and get vehicle role
        String vehicleRole = null;
        if (registration instanceof CarmaMessengerRegistrationMessage) {
            vehicleRole = ((CarmaMessengerRegistrationMessage) registration).getVehicleRole();
            log.info("Receive ns3 registration message of role " + vehicleRole);
            if(ns3RegistrationList.containsKey(vehicleRole)){
                log.error("Duplicate ns3 registration for vehicle {}", vehicleRole);
                return;
            }else{
                ns3RegistrationList.put(vehicleRole, (CarmaMessengerRegistrationMessage)registration);
                log.debug("Added new ns3 registration for vehicle {}", vehicleRole);
            }
            if(bridgeRegistrationList.containsKey(vehicleRole)){
                log.debug("Found bridge registration of vehicle {}, making a carma messenger instance now", vehicleRole);
            }else{
                log.debug("Can't find bridge registration for vehicle {}, waiting for bridge registration", vehicleRole);
                return;
            }           
        } else if (registration instanceof CarmaMessengerBridgeRegistrationMessage) {
            vehicleRole = ((CarmaMessengerBridgeRegistrationMessage) registration).getVehicleRole();
            log.info("Receive bridge registration message of role " + vehicleRole);
            if(bridgeRegistrationList.containsKey(vehicleRole)){
                log.error("Duplicate bridge registration for vehicle {}", vehicleRole);
                return;
            }else{
                bridgeRegistrationList.put(vehicleRole, (CarmaMessengerBridgeRegistrationMessage)registration);
                log.debug("Added new bridge registration for vehicle {}", vehicleRole);
            }
            if(ns3RegistrationList.containsKey(vehicleRole)){
                log.debug("Found ns3 registration of vehicle {}, making a carma messenger instance now", vehicleRole);
            }else{
                log.debug("Can't find ns3 registration for vehicle {}, waiting for ns3 registration", vehicleRole);
                return;
            }
        }
       

        if (!managedInstances.containsKey(vehicleRole)) {
            try {
                // Determine necessary values for creating a new instance
                CarmaMessengerRegistrationMessage regMessage = null;
                CarmaMessengerBridgeRegistrationMessage bridgeMessage = null;
                InetAddress rxV2xAddress = null;
                InetAddress rxBridgeAddress = null;
                int rxV2xTimeSyncPort = 0;
                int rxBridgeTimeSyncPort = 0;

                if (registration instanceof CarmaMessengerRegistrationMessage) {
                    regMessage = (CarmaMessengerRegistrationMessage) registration;
                    bridgeMessage = bridgeRegistrationList.get(vehicleRole);
                    rxV2xAddress = InetAddress.getByName(regMessage.getRxMessageIpAddress());
                    rxBridgeAddress = InetAddress.getByName(bridgeMessage.getRxMessageIpAddress());
                    rxV2xTimeSyncPort = regMessage.getRxTimeSyncPort();
                    rxBridgeTimeSyncPort = bridgeMessage.getRxTimeSyncPort();
                        
                } else if (registration instanceof CarmaMessengerBridgeRegistrationMessage) {
                    bridgeMessage = (CarmaMessengerBridgeRegistrationMessage) registration;
                    regMessage = ns3RegistrationList.get(vehicleRole);
                    rxV2xAddress = InetAddress.getByName(regMessage.getRxMessageIpAddress());
                    rxBridgeAddress = InetAddress.getByName(bridgeMessage.getRxMessageIpAddress());
                    rxBridgeTimeSyncPort = bridgeMessage.getRxTimeSyncPort();
                    rxV2xTimeSyncPort = regMessage.getRxMessagePort();
                }

                    // Initialize parameters for new instance
                String vehicleId = regMessage.getVehicleId();
                int rxMessagePort = regMessage.getRxMessagePort();
                int rxVehicleStatusPort = bridgeMessage.getRxVehicleStatusPort();
                int rxTrafficEventPort = bridgeMessage.getRxTrafficEventPort();

                newCarmaMessengerInstance(vehicleId, 
                                          vehicleRole, 
                                          rxV2xAddress, 
                                          rxBridgeAddress,
                                          rxMessagePort,
                                          rxV2xTimeSyncPort,
                                          rxBridgeTimeSyncPort,
                                          rxVehicleStatusPort,
                                          rxTrafficEventPort
                );

            } catch (Exception e) {
                throw new RuntimeException("Failed to create CarmaMessenger instance", e);
            }
        } else {
            log.warn("Received duplicate registration for vehicle " + vehicleRole);
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
                    log.debug("Sending CARMA Messenger adapter & bridge {} at {}:{} & {}:{} time sync message for time {}!" ,
                        currentInstance.getVehicleId(), 
                        currentInstance.getTargetAddress(), 
                        currentInstance.getTimeSyncPort(),
                        currentInstance.getBridgeAddress(), 
                        currentInstance.getRxBridgeTimeSyncPort(), 
                        message.getTimestep());
                    currentInstance.sendTimeSyncMsg(bytes);
                }
            }
        }

    private void newCarmaMessengerInstance(String carmaMessengerVehId, 
                                           String sumoRoleName, 
                                           InetAddress v2xAddress,
                                           InetAddress bridgeAddress, 
                                           int rxV2xMsgPort, 
                                           int rxV2xTimeSyncPort, 
                                           int rxBridgeTimeSyncPort, 
                                           int rxVehicleStatusPort, 
                                           int rxTrafficEventPort) {
        CarmaMessengerInstance tmp = new CarmaMessengerInstance(carmaMessengerVehId, 
                                                                sumoRoleName, 
                                                                v2xAddress, 
                                                                bridgeAddress, 
                                                                rxV2xMsgPort, 
                                                                rxV2xTimeSyncPort, 
                                                                rxBridgeTimeSyncPort,
                                                                rxVehicleStatusPort, 
                                                                rxTrafficEventPort, 
                                                                false, 
                                                                false);
        try {
            tmp.bind();
            log.info("New CARMA Messenger instance '{}' registered with CARMA Instance Manager.", sumoRoleName);
            log.info(tmp.toString());
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

    public void vehicleStatusUpdate(long updateInterval) throws IOException{
        long updateIntervalSecond = updateInterval / 1000000000L;
        for (String key : managedInstances.keySet()) {           
            CarmaMessengerInstance instance = managedInstances.get(key);
            
            // Get location and status values directly
            GeoPoint prev_location = instance.getPrevLocation();
            log.info("Previous location: {}", prev_location.toCartesian().toString());
            GeoPoint curr_location = instance.getLocation();
            log.info("Current location: {}", curr_location.toCartesian().toString());
        
            // Create the VehicleTwist object with calculated values
            MsgerVehicleStatus.VehicleTwist twist;
            if (prev_location == GeoPoint.ORIGO) {
                // Initialize twist with zeros if it's the origin
                twist = new MsgerVehicleStatus.VehicleTwist(0, 0, 0);
                prev_location = curr_location;
            } else {
                // Directly calculate and assign twist values
                twist = new MsgerVehicleStatus.VehicleTwist(
                    (float) ((curr_location.toCartesian().getX() - prev_location.toCartesian().getX()) / updateIntervalSecond),
                    (float) ((curr_location.toCartesian().getY() - prev_location.toCartesian().getY()) / updateIntervalSecond),
                    (float) ((curr_location.toCartesian().getZ() - prev_location.toCartesian().getZ()) / updateIntervalSecond)
                );
            }
            
            // Create VehiclePose with lat, lon, alt values
            MsgerVehicleStatus.VehiclePose pose = new MsgerVehicleStatus.VehiclePose(
                instance.getLocation().getLatitude(),
                instance.getLocation().getLongitude(),
                instance.getLocation().getAltitude()
            );
        
            // Create the MsgerVehicleStatus object

            MsgerVehicleStatus status = new MsgerVehicleStatus(
                pose,
                twist,
                instance.getSirenActive(),
                instance.getLightActive()
            );
            log.debug(status.toString());
         
            Gson gson = new Gson();
            log.debug("Gson content: {}",gson.toJson(status));
            byte[] bytes = gson.toJson(status).getBytes();
            instance.sendVehStatusMsgs(bytes);
        }
    }

    public void onDetectedTrafficEvents(MsgerTrafficEvent message) throws IOException {
        if (managedInstances.size() == 0) {
            log.debug("There are no registered instances");
        }
        else {
            Gson gson = new GsonBuilder().excludeFieldsWithoutExposeAnnotation().create();
            byte[] bytes = gson.toJson(message).getBytes();
            for (CarmaMessengerInstance currentInstance : managedInstances.values()) {
                log.debug("Instance Role name: {}, message vehicle id: {}", currentInstance.getRoleName(), message.getVehicleId());
                if(currentInstance.getRoleName().equals(message.getVehicleId())){
                    currentInstance.sendTrafficEventMsgs(bytes);
                    log.debug("onDetectedTrafficEvents sent successfully to bridge: {}",message.toString());
                    break;
                }
            }
        }
    }

    @Override
    public void onVehicleUpdates(VehicleUpdates vui) {
        for (VehicleData veh : vui.getUpdated()) {
            if (managedInstances.containsKey(veh.getName())) {
                if(veh.getPosition()== null){
                    return;
                }
                // Save previous Geolocation for the use of twist calculation
                GeoPoint prev_cartesian_location = managedInstances.get(veh.getName()).getLocation();

                // If prev_cartesian_location == GeoPoint.ORIGO, the system just started, set previous geolocation same with current geolocation
                if (prev_cartesian_location == GeoPoint.ORIGO)
                    managedInstances.get(veh.getName()).setPrevLocation(veh.getPosition());
                else
                    managedInstances.get(veh.getName()).setPrevLocation(prev_cartesian_location);

                // Save current Geolocation
                managedInstances.get(veh.getName()).setLocation(veh.getPosition());
            }
        }
    }
}
