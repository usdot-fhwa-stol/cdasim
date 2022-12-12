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

import main.java.org.eclipse.mosaic.fed.carma.ambassador.CarmaRegistrationMessage;

/**
 * Session management class for CARMA Platform instances communicating with MOSAIC
 */
public class CarmaInstanceManager {
    private Map<String, CarmaInstance>  managedInstances = new HashMap<>();
    private double currentSimulationTime;

    // TODO: Verify actual port for CARMA Platform NS-3 adapter
    private static final int TARGET_PORT = 5374;

    public void onNewRegistration(CarmaRegistrationMessage registration) {
        if (!managedInstances.containsKey(txMsg.getVehicleId())) {
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
     * @param rxMsg The V2X Message received
     * @param rxVehicleId The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public void onV2XMessageTx(CarmaV2xMessage txMsg) {
        if (!managedInstances.containsKey(txMsg.getVehicleId()))  {
            return;
        }
    }

    /**
     * Callback to be invoked when CARMA Platform receives a V2X Message from the NS-3 simulation
     * @param rxMsg The V2X Message received
     * @param rxVehicleId The Host ID of the vehicle receiving the data
     * @throws RuntimeException If the socket used to communicate with the platform experiences failure
     */
    public void onV2XMessageRx(CarmaV2xMessage rxMsg, String rxVehicleId) {
        if (!managedInstances.containsKey(rxMsg.getVehicleId()))  {
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
