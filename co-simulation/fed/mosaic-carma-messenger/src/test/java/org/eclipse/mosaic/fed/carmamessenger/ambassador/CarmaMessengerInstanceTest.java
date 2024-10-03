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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.internal.util.reflection.FieldSetter;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceTest {
    /**
     * Mock Datagram socket
     */
    private DatagramSocket socket;
    private CarmaMessengerInstance instance;
     /**
      * Mock InetAddress
      */
    private InetAddress address;

    @Before
    public void setup() throws NoSuchFieldException {
        //init mocks
        address = mock(InetAddress.class);
        socket = mock(DatagramSocket.class);

        //init infra instance
        instance = new CarmaMessengerInstance(
            "MockID",
            "MockRolename",
            address,
            3456,
            7890,
            "MockState"
        );
        FieldSetter.setField(instance, instance.getClass().getDeclaredField("rxMsgsSocket"), socket);
    }

    @Test
    public void testGetterSetterConstructor() {
        assertEquals("MockID", instance.getCarmaVehicleId());
        assertEquals(GeoPoint.ORIGO, instance.getLocation());
        assertEquals(3456, instance.getV2xPort());
        assertEquals(7890, instance.getTimeSyncPort());
        assertEquals("MockState", instance.getMessengerEmergencyState());

        instance.setCarmaVehicleId("NewID");
        assertEquals("NewID", instance.getCarmaVehicleId());
        instance.setV2xPort(1234);
        assertEquals(1234, instance.getV2xPort());
        instance.setTimeSyncPort(5678);
        assertEquals(5678, instance.getTimeSyncPort());
        instance.setTargetAddress(address);
        assertEquals(address, instance.getTargetAddress());
        instance.setMessengerEmergencyState("NewState");
        assertEquals("NewState", instance.getMessengerEmergencyState());

    }

    @Test
    public void testSendV2xMsg() throws IOException {
        // Test SendV2xMsg method
        String test_msg = "test message";
        instance.sendV2xMsgs(test_msg.getBytes());
        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());

        // Verify parameter members
        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getV2xPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }

    @Test
    public void testSendTimeSyncMsg() throws IOException {
        // Test SendTimeSyncMsg method
        TimeSyncMessage test_msg = new TimeSyncMessage(1,100);
        Gson gson = new Gson();
        byte[] bytes = gson.toJson(test_msg).getBytes();
        instance.sendTimeSyncMsg(bytes);

        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());
        // Convert message to bytes
        byte[] message_bytes = gson.toJson(test_msg).getBytes();
        // Verify parameter members
        assertArrayEquals(message_bytes, packet.getValue().getData());
        assertEquals(instance.getTimeSyncPort(), packet.getValue().getPort());
        assertEquals(address, packet.getValue().getAddress());
    }
}
