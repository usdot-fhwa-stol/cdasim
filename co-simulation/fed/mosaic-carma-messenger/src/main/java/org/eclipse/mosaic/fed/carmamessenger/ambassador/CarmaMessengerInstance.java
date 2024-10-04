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
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.eclipse.mosaic.fed.carma.ambassador.CarmaInstance;


public class CarmaMessengerInstance extends CarmaInstance{
    private String carmaMessengerVehicleId;
    private String sumoRoleName;

    private DatagramSocket rxMsgsSocket = null;

    private String messengerEmergencyState;
    private int rxBridgeMessagePort;

    public CarmaMessengerInstance(String carmaMessengerVehicleId, String sumoRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort, String messengerEmergencyState, int rxBridgeMessagePort) {
        super(carmaMessengerVehicleId, sumoRoleName, targetAddress, v2xPort, timeSyncPort);

        this.carmaMessengerVehicleId = carmaMessengerVehicleId;
        this.sumoRoleName = sumoRoleName;

        this.messengerEmergencyState = messengerEmergencyState;
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    public String getCarmaMessengerVehicleId() {
        return carmaMessengerVehicleId;
    }

    public void setCarmaMessengerVehicleId(String carmaMessengerVehicleId) {
        this.carmaMessengerVehicleId = carmaMessengerVehicleId;
    }

    public String getSumoRoleName() {
        return sumoRoleName;
    }

    public void setSumoRoleName(String sumoRoleName) {
        this.sumoRoleName = sumoRoleName;
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

    public int getRxBridgeMessagePort(){
        return rxBridgeMessagePort;
    }

    public void setRxBridgeMessagePort(int rxBridgeMessagePort){
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    /**
     * Sends the V2X message to the CARMA Platform communications interface configured at construction time.
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendV2xMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), super.getV2xPort());

        rxMsgsSocket.send(packet);
    }
    /**
     * Sends the time sync messages to the CARMA Platform to synchronize ros clock with simulation clock.
     * @param data The binary data encoding of json time sync message
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendTimeSyncMsg(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), super.getTimeSyncPort());

        rxMsgsSocket.send(packet);
    }

    /**
     * Sends the V2X message to the CARMA Platform communications interface configured at construction time.
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendVehStatusMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxBridgeMessagePort);

        rxMsgsSocket.send(packet);
    }
}
