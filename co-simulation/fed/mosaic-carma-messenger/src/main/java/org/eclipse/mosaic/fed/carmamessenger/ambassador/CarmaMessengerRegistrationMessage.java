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

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonRegistrationMessage;

public class CarmaMessengerRegistrationMessage extends CommonRegistrationMessage{

    private String messengerEmergencyState;
    private int rxBridgeMessagePort;

    public CarmaMessengerRegistrationMessage(String carmaMessengerVehicleId, String sumoVehicleRole, String rxMessageIpAddress,
            int rxMessagePort, int rxTimeSyncPort, String messengerEmergencyState, int rxBridgeMessagePort) {
        super(carmaMessengerVehicleId, sumoVehicleRole, rxMessageIpAddress, rxMessagePort, rxTimeSyncPort);

        this.messengerEmergencyState = messengerEmergencyState;
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    public String getMessengerEmergencyState() {
        return messengerEmergencyState;
    }

    public void setMessengerEmergencyState(String messengerEmergencyState) {
        this.messengerEmergencyState = messengerEmergencyState;
    }

    public int getRxBridgeMessagePort(){
        return rxBridgeMessagePort;
    }

    public void setRxBridgeMessagePort(int rxBridgeMessagePort){
        this.rxBridgeMessagePort = rxBridgeMessagePort;
    }

    @Override
    public String toString() {
        return "CarmaMessengerRegistrationMessage [carmaMessengerVehicleId=" + super.getVehicleId() + ", sumoVehicleRole=" + super.getVehicleRole()
                + ", rxMessageIpAddress=" + super.getRxMessageIpAddress() + ", rxMessagePort=" + super.getRxMessagePort()
                + ", rxTimeSyncPort=" + super.getRxTimeSyncPort() + ", MessengerEmergencyState=" + messengerEmergencyState + ", rxBridgeMessagePort=" + rxBridgeMessagePort +"]";
    }
}
