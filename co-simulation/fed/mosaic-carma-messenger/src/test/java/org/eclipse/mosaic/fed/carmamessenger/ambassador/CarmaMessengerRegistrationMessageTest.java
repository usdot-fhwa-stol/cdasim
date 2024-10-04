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

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class CarmaMessengerRegistrationMessageTest {
    @Test
    public void testGetterSettersConstructor() {

        CarmaMessengerRegistrationMessage message = new CarmaMessengerRegistrationMessage(
                "MockID",
                "MockRole",
                "127.0.0.1",
                5678,
                1234,
                "MockState",
                5600);
        // Test Getter
        assertEquals("MockID", message.getCarmaMessengerVehicleId());
        assertEquals("MockRole", message.getSumoVehicleRole());

        assertEquals("127.0.0.1", message.getRxMessageIpAddress());
        assertEquals(5678, message.getRxMessagePort());
        assertEquals(1234, message.getRxTimeSyncPort());
        message.setRxMessagePort(5555);
        message.setRxTimeSyncPort(6666);
        message.setCarmaMessengerVehicleId("SOMEID");
        message.setSumoVehicleRole("SOMEROLL");
        message.setRxMessageIpAddress("someIP");
        message.setMessengerEmergencyState("NewState");
        assertEquals("SOMEID", message.getCarmaMessengerVehicleId());
        assertEquals("SOMEROLL", message.getSumoVehicleRole());

        assertEquals("someIP", message.getRxMessageIpAddress());
        assertEquals(5555, message.getRxMessagePort());
        assertEquals(6666, message.getRxTimeSyncPort());
        assertEquals("NewState", message.getMessengerEmergencyState());
        assertEquals(5600, message.getRxBridgeMessagePort());
        message.setRxBridgeMessagePort(5700);
        assertEquals(5700, message.getRxBridgeMessagePort());
    }
}
