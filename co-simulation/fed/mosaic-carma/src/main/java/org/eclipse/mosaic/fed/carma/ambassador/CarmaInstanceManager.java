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
import gov.dot.fhwa.saxton.TimeSyncMessage;

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

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonInstanceManager;

/**
 * Session management class for CARMA Platform instances communicating with MOSAIC
 */
public class CarmaInstanceManager extends CommonInstanceManager<CarmaInstance, CarmaRegistrationMessage>{
    
    public CarmaInstanceManager(){
        setTargetPort(5374);
    }

    /**
     * Helper function to configure a new CARMA Platform instance object upon registration
     * @param carmaVehId The CARMA Platform vehicle ID (e.g. it's license plate number)
     * @param carlaRoleName The Role Name associated with the CARMA Platform's ego vehicle in CARLA
     * @param targetAddress The IP address to which received simulated V2X messages should be sent
     * @param v2xPort The port to which received simulated V2X messages should be sent
     * @param timeSyncPort The port to which to send time sync messages.
     */
    @Override
    protected void newCommonInstance(String carmaVehId, String carlaRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort) {
        CarmaInstance tmp = new CarmaInstance(carmaVehId, carlaRoleName, targetAddress, v2xPort, timeSyncPort);
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

}
