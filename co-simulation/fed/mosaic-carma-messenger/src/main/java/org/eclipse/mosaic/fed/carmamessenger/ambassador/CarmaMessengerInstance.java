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

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonInstance;


public class CarmaMessengerInstance extends CommonInstance{

    private int rxBridgeMessagePort;
    private int rxVehicleStatusPort;
    private int rxTrafficEventPort;


    public CarmaMessengerInstance(String carmaMessengerVehicleId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, int rxBridgeMessagePort, int rxVehicleStatusPort, int rxTrafficEventPort) {
        super(carmaMessengerVehicleId, sumoRoleName, targetAddress, v2xPort, timeSyncPort);
        this.rxBridgeMessagePort = rxBridgeMessagePort;
        this.rxVehicleStatusPort = rxVehicleStatusPort;
        this.rxTrafficEventPort = rxTrafficEventPort;
    }

    public int getRxBridgeMessagePort() {
        return rxBridgeMessagePort;
    }

    public void setRxBridgeMessagePort(int rxBridgeMessagePort) {
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    public int getRxVehicleStatusPort() {
        return rxVehicleStatusPort;
    }
    public int getRxTrafficEventPort() {
        return rxTrafficEventPort;
    }

    public void setRxVehicleStatusPort(int rxVehicleStatusPort) {
        this.rxVehicleStatusPort = rxVehicleStatusPort;
    }

    public void setRxTrafficEventPort(int rxTrafficEventPort) {
        this.rxTrafficEventPort = rxTrafficEventPort;
    }

    public void sendVehStatusMsgs(byte[] data) throws IOException {
        if (super.rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxBridgeMessagePort);
        super.rxMsgsSocket.send(packet);
    }

    public void sendTrafficEventMsgs(byte[] data) throws IOException {
        if (super.rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxTrafficEventPort);
        super.rxMsgsSocket.send(packet);
    }
}
