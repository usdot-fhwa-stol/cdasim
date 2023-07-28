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
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.junit.Before;
import org.junit.Test;
import org.mockito.internal.util.reflection.FieldSetter;

public class InfrastructureInstanceTest {

    private DatagramSocket socket;

    private InfrastructureInstance instance;

    private InetAddress address;

    @Before
    public void setup()  throws NoSuchFieldException{
        address = mock(InetAddress.class);
        socket = mock(DatagramSocket.class);
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
            new Detector(
                "sensor1", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 0.0,0.0,0.0),
                CartesianPoint.ORIGO));
        sensors.add(
            new Detector(
                "NewSensor", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 20.0,20.0,0.0),
                CartesianPoint.ORIGO));
        instance = new InfrastructureInstance(
                "SomeID", 
                address, 
                3456, 
                5667, 
                8888, 
                CartesianPoint.ORIGO, 
                sensors);
        FieldSetter.setField(instance, instance.getClass().getDeclaredField("rxMsgsSocket"), socket);

    }
    @Test
    public void testContainsSensor() {
        assertTrue(instance.containsSensor("NewSensor"));
        assertTrue(instance.containsSensor("sensor1"));
        assertFalse(instance.containsSensor("otherSensor"));
    }

    @Test
    public void testGetterSetterConstructor() {
        assertEquals(instance.getInfrastructureId(), "SomeID");
        assertEquals(instance.getLocation(), CartesianPoint.ORIGO);
        assertEquals(instance.getRxMessagePort(), 3456);
        assertEquals(instance.getTimeSyncPort(), 5667);
    }
}
