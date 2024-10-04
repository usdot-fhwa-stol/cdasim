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

import org.eclipse.mosaic.fed.carma.ambassador.CarmaRegistrationMessage;

public class CarmaMessengerRegistrationMessage extends CarmaRegistrationMessage{
    private String carmaMessengerVehicleId;
    private String sumoVehicleRole;
    private String rxMessageIpAddress;
    private int rxMessagePort;
    private int rxTimeSyncPort;
    private String messengerEmergencyState;

    public CarmaMessengerRegistrationMessage(String carmaMessengerVehicleId, String sumoVehicleRole, String rxMessageIpAddress,
            int rxMessagePort, int rxTimeSyncPort, String messengerEmergencyState) {
        super(carmaMessengerVehicleId, sumoVehicleRole, rxMessageIpAddress, rxMessagePort, rxTimeSyncPort);
        this.carmaMessengerVehicleId = carmaMessengerVehicleId;
        this.sumoVehicleRole = sumoVehicleRole;

        this.messengerEmergencyState = messengerEmergencyState;
    }

    public String getCarmaMessengerVehicleId() {
        return carmaMessengerVehicleId;
    }

    public void setCarmaMessengerVehicleId(String carmaMessengerVehicleId) {
        this.carmaMessengerVehicleId = carmaMessengerVehicleId;
    }

    public String getSumoVehicleRole() {
        return sumoVehicleRole;
    }

    public void setSumoVehicleRole(String carlaVehicleRole) {
        this.sumoVehicleRole = carlaVehicleRole;
    }

    public String getMessengerEmergencyState() {
        return messengerEmergencyState;
    }

    public void setMessengerEmergencyState(String messengerEmergencyState) {
        this.messengerEmergencyState = messengerEmergencyState;
    }

    @Override
    public String toString() {
        return "CarmaMessengerRegistrationMessage [carmaMessengerVehicleId=" + carmaMessengerVehicleId + ", sumoVehicleRole=" + sumoVehicleRole
                + ", rxMessageIpAddress=" + rxMessageIpAddress + ", rxMessagePort=" + rxMessagePort
                + ", rxTimeSyncPort=" + rxTimeSyncPort + ", MessengerEmergencyState=" + messengerEmergencyState + "]";
    }
}
