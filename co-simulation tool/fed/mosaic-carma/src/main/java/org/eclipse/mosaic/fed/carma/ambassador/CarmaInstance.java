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
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

public class CarmaInstance {
    private String vehicleId;
    private double posX;
    private double posY;
    private double lastUpdateTime;

    private DatagramSocket rxMsgsSocket = null;
    private InetAddress targetAddress;
    private int port;

    public CarmaInstance(String vehicleId, double posX, double posY, double lastUpdateTime) {
        this.vehicleId = vehicleId;
        this.posX = posX;
        this.posY = posY;
        this.lastUpdateTime = lastUpdateTime;
    }

    public void connect(InetAddress targetAddress, int port) throws SocketException {
        if (rxMsgsSocket == null) {
            rxMsgsSocket = new DatagramSocket();
            this.targetAddress = targetAddress;
            this.port = port;
        }
    }

    public void sendMsgs(byte[] data) throws IOException {
        if (rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }

        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, port);

        rxMsgsSocket.send(packet);
    }

    public String getVehicleId() {
        return vehicleId;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }

    public void setPosX(double posX) {
        this.posX = posX;
    }

    public void setPosY(double posY) {
        this.posY = posY;
    }

    public void setLastUpdateTime(double lastUpdateTime) {
        this.lastUpdateTime = lastUpdateTime;
    }
}
