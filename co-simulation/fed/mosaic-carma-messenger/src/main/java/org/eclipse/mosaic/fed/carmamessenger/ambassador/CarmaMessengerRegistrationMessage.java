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

public class CarmaMessengerRegistrationMessage {
    private String carmaVehicleId;
    private String carlaVehicleRole;
    private String rxMessageIpAddress;
    private int rxMessagePort;
    private int rxTimeSyncPort;
    private String messengerEmergencyState;

    public CarmaMessengerRegistrationMessage(String carmaVehicleId, String carlaVehicleRole, String rxMessageIpAddress,
            int rxMessagePort, int rxTimeSyncPort, String messengerEmergencyState) {
        this.carmaVehicleId = carmaVehicleId;
        this.carlaVehicleRole = carlaVehicleRole;
        this.rxMessageIpAddress = rxMessageIpAddress;
        this.rxMessagePort = rxMessagePort;
        this.rxTimeSyncPort = rxTimeSyncPort;
        this.messengerEmergencyState = messengerEmergencyState;
    }

    public String getCarmaVehicleId() {
        return carmaVehicleId;
    }

    public void setCarmaVehicleId(String carmaVehicleId) {
        this.carmaVehicleId = carmaVehicleId;
    }

    public String getCarlaVehicleRole() {
        return carlaVehicleRole;
    }

    public void setCarlaVehicleRole(String carlaVehicleRole) {
        this.carlaVehicleRole = carlaVehicleRole;
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

    public String getMessengerEmergencyState() {
        return messengerEmergencyState;
    }

    public void setMessengerEmergencyState(String messengerEmergencyState) {
        this.messengerEmergencyState = messengerEmergencyState;
    }

    @Override
    public String toString() {
        return "CarmaMessengerRegistrationMessage [carmaVehicleId=" + carmaVehicleId + ", carlaVehicleRole=" + carlaVehicleRole
                + ", rxMessageIpAddress=" + rxMessageIpAddress + ", rxMessagePort=" + rxMessagePort
                + ", rxTimeSyncPort=" + rxTimeSyncPort + ", MessengerEmergencyState=" + messengerEmergencyState + "]";
    }
}
