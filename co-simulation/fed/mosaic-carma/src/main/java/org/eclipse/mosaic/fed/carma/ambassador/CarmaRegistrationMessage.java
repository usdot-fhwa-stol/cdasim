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

/**
 * JSON compatible message to be sent by CARMA Platform when it registers with the carma-mosaic ambassador
 */
public class CarmaRegistrationMessage {
    private String carmaVehicleId;
    private String carlaVehicleRole;
    private String rxMessageIpAddress;
    private int rxMessagePort;

    public CarmaRegistrationMessage(String carmaVehicleId, String carlaVehicleRole, String rxMessageIpAddress,
            int rxMessagePort) {
        this.carmaVehicleId = carmaVehicleId;
        this.carlaVehicleRole = carlaVehicleRole;
        this.rxMessageIpAddress = rxMessageIpAddress;
        this.rxMessagePort = rxMessagePort;
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

    @Override
    public String toString() {
        return "CarmaRegistrationMessage [carmaVehicleId=" + carmaVehicleId + ", carlaVehicleRole=" + carlaVehicleRole
                + ", rxMessageIpAddress=" + rxMessageIpAddress + ", rxMessagePort=" + rxMessagePort + "]";
    }
    
}
