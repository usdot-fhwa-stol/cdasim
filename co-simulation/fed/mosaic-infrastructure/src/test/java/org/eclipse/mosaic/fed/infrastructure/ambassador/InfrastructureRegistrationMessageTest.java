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
package org.eclipse.mosaic.fed.infrastructure.ambassador;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.junit.Test;

public class InfrastructureRegistrationMessageTest {
    @Test
    public void testGetterSettersConstructor() {
        // Test Constructor
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
            new Detector(
                "String sensorId", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 0.0,0.0,0.0),
                CartesianPoint.ORIGO));
        InfrastructureRegistrationMessage message = new InfrastructureRegistrationMessage(
                "127.0.0.1",
                 "rsu_1", 
                 4567 , 
                 5678, 
                 8642, 
                 CartesianPoint.xy(1, 2), 
                 sensors);
        // Test Getter
        assertEquals("rsu_1",message.getInfrastructureId());
        assertEquals("127.0.0.1", message.getInfrastructureId() );
        assertEquals(4567, message.getRxMessagePort());
        assertEquals(5678, message.getTimeSyncPort());
        assertEquals(8642, message.getSimulatedInteractionPort());
        assertEquals(CartesianPoint.xy(1,2), message.getLocation());
        assertEquals(sensors, message.getSensors());

    }

}
