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
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.internal.util.reflection.FieldSetter;

import com.google.gson.Gson;

public class InfrastructureInstanceTest {
    /**
     * Mock Datagram socket
     */
    private DatagramSocket socket;

    private InfrastructureInstance instance;
    /**
     * Mock InetAddress
     */
    private InetAddress address;

    @Before
    public void setup() throws NoSuchFieldException {
        // Initialize Mocks
        address = mock(InetAddress.class);
        socket = mock(DatagramSocket.class);
        // Initialize Infrastructure Instance
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
                new Detector(
                        "sensor1",
                        DetectorType.SEMANTIC_LIDAR,
                        new Orientation(0.0, 0.0, 0.0),
                        CartesianPoint.ORIGO));
        sensors.add(
                new Detector(
                        "NewSensor",
                        DetectorType.SEMANTIC_LIDAR,
                        new Orientation(20.0, 20.0, 0.0),
                        CartesianPoint.ORIGO));
        instance = new InfrastructureInstance(
                "SomeID",
                address,
                3456,
                5667,
                8888,
                CartesianPoint.ORIGO,
                sensors);
        // Set private instance field to mock using reflection
        FieldSetter.setField(instance, instance.getClass().getDeclaredField("socket"), socket);

    }

    @Test
    public void testContainsSensor() {
        // Test contains sensor method
        assertTrue(instance.containsSensor("NewSensor"));
        assertTrue(instance.containsSensor("sensor1"));
        assertFalse(instance.containsSensor("otherSensor"));
    }

    @Test
    public void testGetterSetterConstructor() {
        // Test getters and constructor for setting and retrieving class members
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
                        new Orientation(1.0, 2.0, 3.0),
                        CartesianPoint.ORIGO));
        sensors.add(
                new Detector(
                        "NewSensor",
                        DetectorType.SEMANTIC_LIDAR,
                        new Orientation(24.0, 25.0, 6.0),
                        CartesianPoint.ORIGO));
        instance.setSensors(sensors);
        assertEquals(sensors, instance.getSensors());
        instance.setTargetAddress(address);
        assertEquals(address, instance.getTargetAddress());

    }

    @Test
    public void testSendV2xMsg() throws IOException {
        // Test SendV2xMsg method
        String test_msg = "test message";
        instance.sendV2xMsg(test_msg.getBytes());
        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());

        // Verify parameter members
        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getRxMessagePort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

    @Test
    public void testSendTimeSyncMsg() throws IOException {
        // Test SendTimeSyncMsg method
        InfrastructureTimeMessage test_msg = new InfrastructureTimeMessage();
        test_msg.setSeq(1);
        test_msg.setTimestep(100);
        instance.sendTimeSyncMsg(test_msg);


        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());
        // Convert message to bytes
        Gson gson = new Gson();
        byte[] message_bytes = gson.toJson(test_msg).getBytes();
        // Verify parameter members
        assertArrayEquals(message_bytes, packet.getValue().getData());
        assertEquals(instance.getTimeSyncPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

    @Test
    public void testSendInteraction() throws IOException {
        // Test SendInteraction method
        DetectedObject test_msg = new DetectedObject(
                DetectionType.BUS,
                0.5,
                "sensor1",
                "projection String",
                "Object1",
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(0, 0, 0),
                new Vector3d(),
                new Size(0, 0, 0));
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        test_msg.setPositionCovariance(covarianceMatrix);
        test_msg.setVelocityCovariance(covarianceMatrix);
        test_msg.setAngularVelocityCovariance(covarianceMatrix);
        instance.sendDetection(test_msg);
        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());
        // Convert message to bytes
        Gson gson = new Gson();
        byte[] message_bytes = gson.toJson(test_msg).getBytes();
        // Verify parameter members
        assertArrayEquals(message_bytes, packet.getValue().getData());
        assertEquals(instance.getSimulatedInteractionPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

}
