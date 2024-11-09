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
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;
import org.eclipse.mosaic.lib.objects.vehicle.MsgerVehicleStatus;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.transform.Proj4Projection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManager extends CommonInstanceManager<CarmaMessengerInstance, CarmaMessengerRegistrationMessage>{

    private static final int BRIDGE_TARGET_PORT = 5500;
    private final Logger log = LoggerFactory.getLogger(this.getClass());
    private final Map<String, Object> registrationList = new HashMap<>();

    /**
     * Callback to invoked when a new CARMA Platform instance registers with the mosaic-carma ambassador for the first time
     * @param registration The new instance's registration data
     */
    public void onMsgrNewRegistration(Object registration) {
        super.setTargetPort(5600);
        // Check registration type and get vehicle role
        String vehicleRole = null;
        if (registration instanceof CarmaMessengerRegistrationMessage) {
            vehicleRole = ((CarmaMessengerRegistrationMessage) registration).getVehicleRole();
            log.info("Receive ns3 registration message of role " + vehicleRole);
        } else if (registration instanceof CarmaMessengerBridgeRegistrationMessage) {
            vehicleRole = ((CarmaMessengerBridgeRegistrationMessage) registration).getVehicleRole();
            log.info("Receive bridge registration message of role " + vehicleRole);
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
                    int rxBridgeTimeSyncPort = bridgeMessage.getRxTimeSyncPort();
                    int rxVehicleStatusPort = bridgeMessage.getRxVehicleStatusPort();
                    int rxTrafficEventPort = bridgeMessage.getRxTrafficEventPort();

                    newCarmaMessengerInstance(
                        vehicleId, vehicleRole, rxMessageIp, rxMessagePort, 
                        rxTimeSyncPort, rxBridgeTimeSyncPort,
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
     * Helper function to configure and register a new CARMA Messenger instance object.
     *
     * @param carmaMessengerVehId   The unique CARMA Messenger vehicle ID, typically the vehicle's identifier 
     *                              or license plate number.
     * @param sumoRoleName          The role name associated with the CARMA Messenger ego vehicle in SUMO, used
     *                              to distinguish it within the simulation environment.
     * @param targetAddress         The IP address where received simulated V2X (Vehicle-to-Everything) messages 
     *                              will be forwarded.
     * @param v2xPort               The port number designated for sending simulated V2X messages to the target address.
     * @param timeSyncPort          The port number for sending time synchronization messages to keep the CARMA 
     *                              Platform instance in sync with other components.
     * @param rxBridgeMessagePort   The port used to receive bridge messages from external systems or 
     *                              simulation environments.
     * @param rxVehicleStatusPort   The port for receiving vehicle status updates, such as position and 
     *                              movement data, from the CARMA Platform's ego vehicle.
     * @param rxTrafficEventPort    The port designated for receiving traffic event information that 
     *                              may impact the CARMA Platform's operation (e.g., road hazards or alerts).
     */
    private void newCarmaMessengerInstance(String carmaMessengerVehId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, int rxBridgeMessagePort, int rxVehicleStatusPort, int rxTrafficEventPort) {
        CarmaMessengerInstance tmp = new CarmaMessengerInstance(carmaMessengerVehId, sumoRoleName, targetAddress, v2xPort, timeSyncPort, rxBridgeMessagePort,rxVehicleStatusPort, rxTrafficEventPort, false, false);
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
        for (String key : managedInstances.keySet()) {
            CarmaMessengerInstance instance = managedInstances.get(key);
            
            // Get location and status values directly
            GeoPoint prev_location = instance.getPrevLocation();
            GeoPoint curr_location = instance.getLocation();
        
            // Create the VehicleTwist object with calculated values
            MsgerVehicleStatus.VehicleTwist twist;
            if (prev_location == GeoPoint.ORIGO) {
                // Initialize twist with zeros if it's the origin
                twist = new MsgerVehicleStatus.VehicleTwist(0, 0, 0);
            } else {
                // Directly calculate and assign twist values
                twist = new MsgerVehicleStatus.VehicleTwist(
                    (float) ((curr_location.toCartesian().getX() - prev_location.toCartesian().getX()) / updateInterval),
                    (float) ((curr_location.toCartesian().getY() - prev_location.toCartesian().getY()) / updateInterval),
                    (float) ((curr_location.toCartesian().getZ() - prev_location.toCartesian().getZ()) / updateInterval)
                );
            }
            
            // Create VehiclePose with lat, lon, alt values
            MsgerVehicleStatus.VehiclePose pose = new MsgerVehicleStatus.VehiclePose(
                instance.getGeoLocation().getLatitude(),
                instance.getGeoLocation().getLongitude(),
                instance.getGeoLocation().getAltitude()
            );
        
            // Create the MsgerVehicleStatus object
            MsgerVehicleStatus status = new MsgerVehicleStatus(
                pose,
                twist,
                instance.getSirenActive(),
                instance.getLightActive()
            );
            Gson gson = new GsonBuilder().excludeFieldsWithoutExposeAnnotation().create();
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
                if(currentInstance.getRoleName().equals(message.getVehicleId())){
                    currentInstance.sendTrafficEventMsgs(bytes);
                    log.debug(message.toString());
                    break;
                }
            }
        }
    }

    @Override
    public void onVehicleUpdates(VehicleUpdates vui) {
        for (VehicleData veh : vui.getUpdated()) {
            if (managedInstances.containsKey(veh.getName())) {
                
                // Save previous cartesian point for the use of twist calculation
                GeoPoint prev_cartesian_location = managedInstances.get(veh.getName()).getLocation();
                managedInstances.get(veh.getName()).setPrevLocation(prev_cartesian_location);

                // Save new cartesian point
                managedInstances.get(veh.getName()).setLocation(veh.getPosition());

                // Converting cartesian point to geo location
                Proj4Projection proj = new Proj4Projection(veh.getPosition(), 0, 0, null);
                MutableGeoPoint temp = new MutableGeoPoint();
                MutableGeoPoint location = proj.cartesianToGeographic(veh.getProjectedPosition(), temp);
                GeoPoint result = GeoPoint.latLon(location.getLatitude(), location.getLongitude());

                // Save converted geo location to the instance
                managedInstances.get(veh.getName()).setGeoLocation(result);
            }
        }
    }
}
