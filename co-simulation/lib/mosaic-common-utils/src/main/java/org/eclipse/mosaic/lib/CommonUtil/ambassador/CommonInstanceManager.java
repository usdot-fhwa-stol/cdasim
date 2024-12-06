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
package org.eclipse.mosaic.lib.CommonUtil.ambassador;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

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

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.TimeSyncMessage;


public class CommonInstanceManager<T extends CommonInstance, R extends CommonRegistrationMessage> {
    protected  Map<String, T> managedInstances = new HashMap<>();

    private int TARGET_PORT = 5374;
    protected final Logger log = LoggerFactory.getLogger(this.getClass());

    public void onNewRegistration(R registration) {
        if (!managedInstances.containsKey(registration.getVehicleRole())) {
            try {
                newCommonInstance(
                    registration.getVehicleId(),
                    registration.getVehicleRole(),
                    InetAddress.getByName(registration.getRxMessageIpAddress()),
                    registration.getRxMessagePort(),
                    registration.getRxTimeSyncPort()
                );
            } catch (UnknownHostException e) {
                throw new RuntimeException(e);
            }
        } else {
            // log warning
            log.warn("Received duplicate registration for vehicle " + registration.getVehicleRole());
        }
    }

    protected void newCommonInstance(String VehId, String RoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort) {
        CommonInstance tmp = new CommonInstance(VehId, RoleName, targetAddress, v2xPort, timeSyncPort);
        try {
            tmp.bind();
            log.info("New Common instance '{}' registered with CARMA Instance Manager.", RoleName);
        } catch (IOException e) {
            log.error("Failed to bind Common instance with ID '{}' to its RX message socket: {}",
            RoleName, e.getMessage());
            log.error("Stack trace:", e);
            throw new RuntimeException(e);
        }
        managedInstances.put(RoleName, (T) tmp);
    }

    public boolean checkIfRegistered(String mosiacVehicleId) {
        return managedInstances.keySet().contains(mosiacVehicleId);
    }
    
    public V2xMessageTransmission onV2XMessageTx(InetAddress sourceAddr, CarmaV2xMessage txMsg, long time) {
        T sender = null;
        // Find the CarmaInstance with sourceAddr.
        for (T ci : managedInstances.values()) {
            if (ci.getTargetAddress().equals(sourceAddr)) {
                sender = ci;
                break;
            }
        }
        // Unregistered instance attempting to send messages
        if (sender == null) {
            throw new IllegalStateException("Unregistered Common instance attempting to send messages via MOSAIC");
        }
        AdHocMessageRoutingBuilder messageRoutingBuilder = new AdHocMessageRoutingBuilder(
                sender.getRoleName(), sender.getLocation()).viaChannel(AdHocChannel.CCH);
        // TODO: Get maximum broadcast radius from configuration file.
        MessageRouting routing = messageRoutingBuilder.geoBroadCast(new GeoCircle(sender.getLocation(), 300));
        log.debug("Generating V2XMessageTransmission interaction sim time: {}, sender id: {}, location: {}, type: {}, payload: {}", 
                time, 
                sender.getVehicleId(),
                sender.getLocation(),
                txMsg.getType(),
                txMsg.getPayload()
            );
        return new V2xMessageTransmission( time, new ExternalV2xMessage(routing,
                new ExternalV2xContent( time, sender.getLocation(), txMsg.getPayload())));
    }

    public void onV2XMessageRx(byte[] rxMsg, String rxVehicleId) {
        if (!managedInstances.containsKey(rxVehicleId))  {
            return;
        }

        T common = managedInstances.get(rxVehicleId);
        try {
            common.sendV2xMsgs(rxMsg);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void onTimeStepUpdate(TimeSyncMessage message) throws IOException {
        if (managedInstances.size() == 0) {
            log.debug("There are no registered instances");
        }
        else {
            Gson gson = new Gson();
            byte[] bytes = gson.toJson(message).getBytes();
            for (T currentInstance : managedInstances.values()) {
                log.debug("Sending Common instance {} at {}:{} time sync message for time {}!" ,
                    currentInstance.getVehicleId(), 
                    currentInstance.getTargetAddress(), 
                    currentInstance.getTimeSyncPort(), 
                    message.getTimestep());
                currentInstance.sendTimeSyncMsg(bytes);
            }
        }
    }

    public int getTargetPort(){
        return TARGET_PORT;
    }

    public void setTargetPort(int targetPort){
        this.TARGET_PORT = targetPort;
    }

    public void onVehicleUpdates(VehicleUpdates vui) {
        for (VehicleData veh : vui.getUpdated()) {
            if (managedInstances.containsKey(veh.getName())) {
                log.debug("On vehicle update: Vehicle lat {} lon {}", veh.getPosition().getLatitude(), veh.getPosition().getLongitude());
                managedInstances.get(veh.getName()).setLocation(veh.getPosition());
            }
        }
    }
}
