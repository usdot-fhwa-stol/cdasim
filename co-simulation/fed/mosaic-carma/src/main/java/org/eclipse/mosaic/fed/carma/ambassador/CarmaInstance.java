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

import org.eclipse.mosaic.lib.geo.GeoPoint;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

/**
 * Connection manager and data object to associate with a single CARMA Platform instance in XIL
 */
public class CarmaInstance {
    private String carmaVehicleId;
    private String carlaRoleName;
    private DatagramSocket rxMsgsSocket = null;

    private InetAddress targetAddress;
    private int v2xPort;
    private int timeSyncPort;
    private GeoPoint location = GeoPoint.ORIGO;

    public CarmaInstance(String carmaVehicleId, String carlaRoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort ) {
        this.carmaVehicleId = carmaVehicleId;
        this.carlaRoleName = carlaRoleName;
        this.targetAddress = targetAddress;
        this.v2xPort = v2xPort;
        this.timeSyncPort = timeSyncPort;
    }

    public InetAddress getTargetAddress() {
        return targetAddress;
    }

    public void setTargetAddress(InetAddress targetAddress) {
        this.targetAddress = targetAddress;
    }

    public void setLocation(GeoPoint location) {
        this.location = location;
    }

    public GeoPoint getLocation() {
        return this.location;
    }

    /**
     * Sends the data to the CARMA Platform communications interface configured at construction time.
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendV2xMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, v2xPort);

        rxMsgsSocket.send(packet);
    }
    /**
     * Sends the data to the CARMA Platform communications interface configured at construction time.
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendTimeSyncMsg(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, timeSyncPort);

        rxMsgsSocket.send(packet);
    }

    /**
     * Connects the sockt to receive messages from the CARMA Platform instance
     * @throws IOException If the socket creation fails
     */
    public void bind() throws IOException {
        rxMsgsSocket = new DatagramSocket();
    }

    public String getCarmaVehicleId() {
        return carmaVehicleId;
    }

    public void setCarmaVehicleId(String carmaVehicleId) {
        this.carmaVehicleId = carmaVehicleId;
    }

    public String getCarlaRoleName() {
        return carlaRoleName;
    }

    public void setCarlaRoleName(String carlaRoleName) {
        this.carlaRoleName = carlaRoleName;
    }
}
