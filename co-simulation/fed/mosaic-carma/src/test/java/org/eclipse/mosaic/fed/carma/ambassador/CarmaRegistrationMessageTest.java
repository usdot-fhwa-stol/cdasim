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
package org.eclipse.mosaic.fed.carma.ambassador;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class CarmaRegistrationMessageTest {
    @Test
    public void testGetterSettersConstructor() {

        CarmaRegistrationMessage message = new CarmaRegistrationMessage(
                "ID",
                "role",
                "127.0.0.1",
                5678,
                8642);
        // Test Getter
        assertEquals("ID", message.getCarmaVehicleId());
        assertEquals("role", message.getCarlaVehicleRole());

        assertEquals("127.0.0.1", message.getRxMessageIpAddress());
        assertEquals(5678, message.getRxMessagePort());
        assertEquals(8642, message.getRxTimeSyncPort());

    }

}
