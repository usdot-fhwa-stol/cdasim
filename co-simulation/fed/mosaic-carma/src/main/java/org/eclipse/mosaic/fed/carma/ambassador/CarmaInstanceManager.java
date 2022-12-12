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

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.fed.carma.ambassador.CarmaRegistrationMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.objects.addressing.AdHocMessageRoutingBuilder;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;

/**
 * Session management class for CARMA Platform instances communicating with MOSAIC
 */
public class CarmaInstanceManager {
    private Map<String, CarmaInstance>  managedInstances = new HashMap<>();
    private double currentSimulationTime;

    // TODO: Verify actual port for CARMA Platform NS-3 adapter
    private static final int TARGET_PORT = 5374;

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
            }
        }

        if (sender == null) {
            // Unregistered instance attempting to send messages
            throw new IllegalStateException("Unregistered CARMA Platform instance attempting to send messages via MOSAIC");
        }

        AdHocMessageRoutingBuilder messageRoutingBuilder = new AdHocMessageRoutingBuilder(
                sender.getCarlaRoleName(), sender.getLocation()).viaChannel(AdHocChannel.CCH);

        MessageRouting routing = messageRoutingBuilder.topoBroadCast(1);

        return new V2xMessageTransmission((long) currentSimulationTime, new ExternalV2xMessage(routing,
                new ExternalV2xContent((long) currentSimulationTime, sender.getLocation(), txMsg.getPayload())));

    }

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
    public void onV2XMessageRx(CarmaV2xMessage rxMsg, String rxVehicleId) {
        if (!managedInstances.containsKey(rxVehicleId))  {
            return;
        }

        CarmaInstance carma = managedInstances.get(rxVehicleId);
        try {
            carma.sendMsgs(rxMsg.encodeV2xMessage());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void newCarmaInstance(String carmaVehId, String carlaRoleName, InetAddress targetAddress, int targetPort) {
        CarmaInstance tmp = new CarmaInstance(carmaVehId, carlaRoleName, targetAddress, targetPort);
        try {
            tmp.bind();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        managedInstances.put(carlaRoleName, tmp);
    }
}
