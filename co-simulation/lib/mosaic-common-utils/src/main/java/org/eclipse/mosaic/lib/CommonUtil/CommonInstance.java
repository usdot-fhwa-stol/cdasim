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
package org.eclipse.mosaic.lib.CommonUtil;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.eclipse.mosaic.lib.geo.GeoPoint;

public class CommonInstance {

    private String VehicleId;
    private String RoleName;
    private InetAddress targetAddress;
    private int v2xPort;
    private int timeSyncPort;
    private GeoPoint location = GeoPoint.ORIGO;
    protected DatagramSocket rxMsgsSocket = null;
    
    public CommonInstance(String VehicleId, String RoleName, InetAddress targetAddress, int v2xPort, int timeSyncPort){
        this.VehicleId = VehicleId;
        this.RoleName = RoleName;
        this.targetAddress = targetAddress;
        this.v2xPort = v2xPort;
        this.timeSyncPort = timeSyncPort;
    }

    public String getVehicleId() {
        return VehicleId;
    }
    
    public void setVehicleId(String VehicleId) {
        this.VehicleId = VehicleId;
    }

    public String getRoleName() {
        return RoleName;
    }

    public void setRoleName(String RoleName) {
        this.RoleName = RoleName;
    }

    public InetAddress getTargetAddress() {
        return targetAddress;
    }
    
    public void setTargetAddress(InetAddress targetAddress) {
        this.targetAddress = targetAddress;
    }

    public int getV2xPort() {
        return v2xPort;
    }
    
    public void setV2xPort(int v2xPort) {
        this.v2xPort = v2xPort;
    }

    public int getTimeSyncPort() {
        return timeSyncPort;
    }
    
    public void setTimeSyncPort(int timeSyncPort) {
        this.timeSyncPort = timeSyncPort;
    }

    public void setLocation(GeoPoint location) {
        this.location = location;
    }

    public GeoPoint getLocation() {
        return this.location;
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

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, v2xPort);

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
}
