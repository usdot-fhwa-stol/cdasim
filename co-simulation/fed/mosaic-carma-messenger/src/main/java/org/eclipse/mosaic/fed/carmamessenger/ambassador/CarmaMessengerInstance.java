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
import org.eclipse.mosaic.lib.geo.GeoPoint;


public class CarmaMessengerInstance extends CommonInstance{

    private int rxBridgeTimeSyncPort;
    private int rxVehicleStatusPort;
    private int rxTrafficEventPort;
    private boolean siren_active;
    private boolean light_active;
    private GeoPoint geo_location;
    private GeoPoint prev_location;
    
    public CarmaMessengerInstance(String carmaMessengerVehicleId, 
                                  String sumoRoleName, 
                                  InetAddress targetAddress, 
                                  int v2xPort, 
                                  int timeSyncPort, 
                                  int rxBridgeTimeSyncPort, 
                                  int rxVehicleStatusPort, 
                                  int rxTrafficEventPort, 
                                  boolean siren_active, 
                                  boolean light_active) {
        super(carmaMessengerVehicleId, sumoRoleName, targetAddress, v2xPort, timeSyncPort);
        this.rxBridgeTimeSyncPort = rxBridgeTimeSyncPort;
        this.rxVehicleStatusPort = rxVehicleStatusPort;
        this.rxTrafficEventPort = rxTrafficEventPort;
        this.siren_active = siren_active;
        this.light_active = light_active;
        this.geo_location = GeoPoint.ORIGO;
        this.prev_location = GeoPoint.ORIGO;
    }

    public boolean getSirenActive(){
        return siren_active;
    }

    public boolean getLightActive(){
        return light_active;
    }

    public int getRxBridgeMessagePort() {
        return rxBridgeTimeSyncPort;
    }

    public void setRxBridgeMessagePort(int rxBridgeTimeSyncPort) {
        this.rxBridgeTimeSyncPort = rxBridgeTimeSyncPort;
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
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxVehicleStatusPort);
        super.rxMsgsSocket.send(packet);
    }

    public void sendTrafficEventMsgs(byte[] data) throws IOException {
        if (super.rxMsgsSocket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        
        DatagramPacket packet = new DatagramPacket(data, data.length, super.getTargetAddress(), rxTrafficEventPort);
        super.rxMsgsSocket.send(packet);
    }

    public GeoPoint getGeoLocation(){
        return this.geo_location;
    }

    public void setGeoLocation(GeoPoint geo_location){
        this.geo_location = geo_location;
    }

    public GeoPoint getPrevLocation(){
        return this.prev_location;
    }

    public void setPrevLocation(GeoPoint prev_cartesian_location){
        this.prev_location = prev_cartesian_location;
    }
}
