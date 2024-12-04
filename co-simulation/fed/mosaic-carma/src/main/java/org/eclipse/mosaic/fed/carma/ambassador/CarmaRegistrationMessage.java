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

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonRegistrationMessage;
/**
 * JSON compatible message to be sent by CARMA Platform when it registers with
 * the carma-mosaic ambassador
 */
public class CarmaRegistrationMessage extends CommonRegistrationMessage{


    public CarmaRegistrationMessage(String carmaVehicleId, String carlaVehicleRole, String rxMessageIpAddress,
            int rxMessagePort, int rxTimeSyncPort) {
                super(carmaVehicleId, carlaVehicleRole, rxMessageIpAddress, rxMessagePort, rxTimeSyncPort);
    }

}
