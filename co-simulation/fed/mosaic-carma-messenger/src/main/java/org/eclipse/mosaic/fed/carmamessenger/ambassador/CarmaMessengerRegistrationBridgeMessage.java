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

public class CarmaMessengerRegistrationBridgeMessage{

    private String sumoVehicleRole;
    private String rxIpAddress;
    private int rxTimeSyncPort;
    private int rxVehicleStatusPort;
    private int rxTrafficEventPort;

    public CarmaMessengerRegistrationBridgeMessage(String sumoVehicleRole, String rxIpAddress, int rxTimeSyncPort, int rxVehicleStatusPort, int rxTrafficEventPort) {
       
        this.sumoVehicleRole = sumoVehicleRole;
        this.rxIpAddress = rxIpAddress;
        this.rxTimeSyncPort = rxTimeSyncPort;
        this.rxVehicleStatusPort = rxVehicleStatusPort;
        this.rxTrafficEventPort = rxTrafficEventPort;
    }

    public String getVehicleRole(){
        return sumoVehicleRole;
    }

    public void setVehicleRole(String vehicleRole){
        this.sumoVehicleRole = vehicleRole;
    }

    public String getRxMessageIpAddress(){
        return rxIpAddress;
    }

    public void setRxMessageIpAddress(String rxIpAddress){
        this.rxIpAddress = rxIpAddress;
    }

    public int getRxTimeSyncPort(){
        return  rxTimeSyncPort;
    }

    public void setRxTimeSyncPort(int rxTimeSyncPort){
        this.rxTimeSyncPort = rxTimeSyncPort;
    }

    public int getVehicleStatusPort(){
        return rxVehicleStatusPort;
    }

    public void setVehicleStatusPort(int rxVehicleStatusPort){
        this.rxVehicleStatusPort = rxVehicleStatusPort;
    }

    public int getTrafficEventPort(){
        return rxTrafficEventPort;
    }

    public void setTrafficEventPort(int rxTrafficEventPort){
        this.rxTrafficEventPort = rxTrafficEventPort;
    }

    @Override
    public String toString() {
        return "CarmaMessengerRegistrationBridgeMessage [sumoVehicleRole=" + sumoVehicleRole
                + ", rxMessageIpAddress=" + rxIpAddress + ", rxTimeSyncPort=" + rxTimeSyncPort + ", VehicleStatusPort=" + rxVehicleStatusPort +", TrafficEventPort"+ rxTrafficEventPort +"]";
    }
}
