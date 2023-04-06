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
 
package org.eclipse.mosaic.fed.infrastructure.ambassador;

import org.eclipse.mosaic.lib.geo.GeoPoint;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

/**
 * Connection manager and data object to associate with a single infrastructure instance in XIL
 * NOTE: TODO See carma.ambassador for reference
 */
public class InfrastructureInstance {
   
    private DatagramSocket rxMsgsSocket = null;
    private InetAddress targetAddress;
    private int registrationPort;
    private int timeSyncPort;
    private GeoPoint location = null;

    public InfrastructureInstance() {
        // TODO
        // TODO Initialize Datagram Socket
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
     * Sends the data to the Infrastructure Device communications interface configured at construction time.
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, registrationPort);
        rxMsgsSocket.send(packet);

    }

    public void sendTimesyncMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, timeSyncPort);
        rxMsgsSocket.send(packet);

    }
}
