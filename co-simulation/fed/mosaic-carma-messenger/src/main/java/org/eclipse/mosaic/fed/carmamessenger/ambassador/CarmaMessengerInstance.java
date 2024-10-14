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
import java.net.DatagramPacket;
import java.net.InetAddress;

import org.eclipse.mosaic.lib.CommonUtil.CommonInstance;


public class CarmaMessengerInstance extends CommonInstance{

    private String messengerEmergencyState;
    private int rxBridgeMessagePort;

    public CarmaMessengerInstance(String carmaMessengerVehicleId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, String messengerEmergencyState, int rxBridgeMessagePort) {
        super(carmaMessengerVehicleId, sumoRoleName, targetAddress, v2xPort, timeSyncPort);
        this.messengerEmergencyState = messengerEmergencyState;
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }
    /**
     * Carma Messenger Emergency state
     */

    public String getMessengerEmergencyState() {
        return messengerEmergencyState;
    }

    public void setMessengerEmergencyState(String messengerEmergencyState) {
        this.messengerEmergencyState = messengerEmergencyState;
    }

    public int getRxBridgeMessagePort() {
        return rxBridgeMessagePort;
    }

    public void setRxBridgeMessagePort(int rxBridgeMessagePort) {
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    public void sendVehStatusMsgs(byte[] data) throws IOException {
        if (super.rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxBridgeMessagePort);
        super.rxMsgsSocket.send(packet);
    }
}
