/*
 * Copyright (C) 2024 LEIDOS.
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

package org.eclipse.mosaic.interactions.msger;
import org.junit.Test;
import static org.junit.Assert.*;

import org.eclipse.mosaic.interactions.application.MsgerRequestTrafficEvent;

public class MsgerRequestTrafficEventTest {

    @Test
    public void testConstructorAndGetters() {
        long time = 100L;
        String vehicleId = "vehicle123";
        String parameterName = "speed";

        MsgerRequestTrafficEvent event = new MsgerRequestTrafficEvent(time, vehicleId, parameterName);

        // Validate constructor and getters
        assertEquals(time, event.getTime());
        assertEquals(vehicleId, event.vehicleId());
        assertEquals(parameterName, event.getParameterName());
    }

    @Test
    public void testEqualsAndHashCode() {
        MsgerRequestTrafficEvent event1 = new MsgerRequestTrafficEvent(100L, "vehicle123", "speed");
        MsgerRequestTrafficEvent event2 = new MsgerRequestTrafficEvent(100L, "vehicle123", "speed");
        MsgerRequestTrafficEvent event3 = new MsgerRequestTrafficEvent(200L, "vehicle456", "position");

        // Test equality
        assertEquals(event1, event2);
        assertNotEquals(event1, event3);

        // Test hash codes
        assertEquals(event1.hashCode(), event2.hashCode());
        assertNotEquals(event1.hashCode(), event3.hashCode());
    }

    @Test
    public void testToString() {
        MsgerRequestTrafficEvent event = new MsgerRequestTrafficEvent(100L, "vehicle123", "speed");

        // Verify toString output
        String expectedString = "MsgerRequestTrafficEvent[vehicleId=vehicle123,Parameter=speed]";
        assertEquals(expectedString, event.toString());
    }
}
