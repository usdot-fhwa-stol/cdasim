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
package org.eclipse.mosaic.lib.util;

public class CommonRegistrationMessage {

    private String vehicleId;
    private String vehicleRole;
    private String rxMessageIpAddress;
    private int rxMessagePort;
    private int rxTimeSyncPort;

    public CommonRegistrationMessage(String vehicleId, String vehicleRole, String rxMessageIpAddress,int rxMessagePort, int rxTimeSyncPort) {
    this.vehicleId = vehicleId;
    this.vehicleRole = vehicleRole;
    this.rxMessageIpAddress = rxMessageIpAddress;
    this.rxMessagePort = rxMessagePort;
    this.rxTimeSyncPort = rxTimeSyncPort;
    }

    public String getVehicleId() {
    return vehicleId;
    }

    public void setVehicleId(String vehicleId) {
    this.vehicleId = vehicleId;
    }

    public String getVehicleRole() {
    return vehicleRole;
    }

    public void setVehicleRole(String vehicleRole) {
    this.vehicleRole = vehicleRole;
    }

    public String getRxMessageIpAddress() {
    return rxMessageIpAddress;
    }

    public void setRxMessageIpAddress(String rxMessageIpAddress) {
    this.rxMessageIpAddress = rxMessageIpAddress;
    }

    public int getRxMessagePort() {
    return rxMessagePort;
    }

    public void setRxMessagePort(int rxMessagePort) {
    this.rxMessagePort = rxMessagePort;
    }

    public int getRxTimeSyncPort() {
    return rxTimeSyncPort;
    }

    public void setRxTimeSyncPort(int rxTimeSyncPort) {
    this.rxTimeSyncPort = rxTimeSyncPort;
    }

    @Override
    public String toString() {
    return "CarmaRegistrationMessage [carmaVehicleId=" + vehicleId + ", carlaVehicleRole=" + vehicleRole
            + ", rxMessageIpAddress=" + rxMessageIpAddress + ", rxMessagePort=" + rxMessagePort
            + ", rxTimeSyncPort=" + rxTimeSyncPort + "]";
    }
    
}
