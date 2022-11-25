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

    /**
     * Callback to invoked when a CARMA Platform vehicle broadcasts a V2X message
     * @param txMsg The message information
     * @return true if that vehicle has already been tracked by this instance manager, false o.w.
     */
    public boolean onV2XMessageTx(CarmaV2xMessage txMsg) {
        if (!managedInstances.containsKey(txMsg.getVehicleId())) {
            newCarmaInstance(
                txMsg.getVehicleId(), 
                txMsg.getVehiclePosX(), 
                txMsg.getVehiclePosY(),
                txMsg.getOriginAddress(),
                TARGET_PORT);
            return false;
        } else {
            updateCarmaInstance(
                txMsg.getVehicleId(), 
                txMsg.getVehiclePosX(), 
                txMsg.getVehiclePosY());
            return true;
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

    private void newCarmaInstance(String vehId, double xPos, double yPos, InetAddress address, int port) {
        CarmaInstance tmp = new CarmaInstance(vehId, xPos, yPos, currentSimulationTime);
        try {
            tmp.connect(address, port);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
        managedInstances.put(vehId, new CarmaInstance(vehId, xPos, yPos, currentSimulationTime));
    }

    private void updateCarmaInstance(String vehId, double xPos, double yPos) {
        managedInstances.get(vehId).setPosX(xPos);
        managedInstances.get(vehId).setPosY(yPos);
        managedInstances.get(vehId).setLastUpdateTime(currentSimulationTime);
    }
}
