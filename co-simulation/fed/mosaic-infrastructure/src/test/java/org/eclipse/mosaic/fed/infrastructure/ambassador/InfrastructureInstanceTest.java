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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
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
        assertEquals("SomeID", instance.getInfrastructureId());
        assertEquals(CartesianPoint.ORIGO, instance.getLocation());
        assertEquals(3456, instance.getRxMessagePort());
        assertEquals(5667, instance.getTimeSyncPort());
        // Test Setter
        instance.setInfrastructureId("DifferentID");
        assertEquals("DifferentID", instance.getInfrastructureId());
        instance.setLocation(CartesianPoint.xy(40, 50));
        assertEquals(CartesianPoint.xy(40, 50), instance.getLocation());
        instance.setTimeSyncPort(4321);
        assertEquals(4321, instance.getTimeSyncPort());
        instance.setRxMessagePort(5678);
        assertEquals(5678, instance.getRxMessagePort());
        instance.setSimulatedInteractionPort(9999);
        assertEquals(9999, instance.getSimulatedInteractionPort());
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
            new Detector(
                "sensor1", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 1.0,2.0,3.0),
                CartesianPoint.ORIGO));
        sensors.add(
            new Detector(
                "NewSensor", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 24.0,25.0,6.0),
                CartesianPoint.ORIGO));
        instance.setSensors(sensors);
        assertEquals(sensors, instance.getSensors());
        instance.setTargetAddress(address);
        assertEquals(address, instance.getTargetAddress());


    }

    @Test
    public void testSendV2xMsg() throws IOException {
        String test_msg = "test message";
        instance.sendV2xMsg(test_msg.getBytes());
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);

        verify(socket, times(1)).send(packet.capture());

        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getRxMessagePort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

    @Test
    public void testSendTimeSyncMsg() throws IOException {
        String test_msg = "test message";
        instance.sendTimeSyncMsg(test_msg.getBytes());
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);

        verify(socket, times(1)).send(packet.capture());

        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getTimeSyncPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

    @Test
    public void testSendInteraction() throws IOException {
        String test_msg = "test message";
        instance.sendInteraction(test_msg.getBytes());
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);

        verify(socket, times(1)).send(packet.capture());

        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getSimulatedInteractionPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }


}
