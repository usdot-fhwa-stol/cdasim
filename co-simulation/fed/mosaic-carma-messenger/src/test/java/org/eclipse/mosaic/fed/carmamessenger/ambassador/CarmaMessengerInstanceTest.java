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

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import org.mockito.internal.util.reflection.FieldSetter;

public class CarmaMessengerInstanceTest{
    /**
     * Mock Datagram socket
     */
    private DatagramSocket socket;
    private CarmaMessengerInstance instance;
     /**
      * Mock InetAddress
      */
    private InetAddress v2xAddress;
    private InetAddress bridgeAddress;

    @Before
    public void setup() throws NoSuchFieldException {
        //init mocks
        v2xAddress = mock(InetAddress.class);
        bridgeAddress = mock(InetAddress.class);
        socket = mock(DatagramSocket.class);

        //init infra instance
        instance = new CarmaMessengerInstance(
            "MockID",
            "MockRolename",
            v2xAddress,
            bridgeAddress,
            3456,
            7890,
            5600,         
            1100,
            1200,
            false,
            false

        ); 
        FieldSetter.setField(instance, instance.getClass().getSuperclass().getDeclaredField("rxMsgsSocket"), socket);
    }

    @SuppressWarnings("deprecation")
    @Test
    public void testGetterSetterConstructor() {
        
        assertEquals(5600, instance.getRxBridgeTimeSyncPort());
        instance.setRxBridgeTimeSyncPort(5700);
        assertEquals(5700, instance.getRxBridgeTimeSyncPort());

        assertEquals(1100, instance.getRxVehicleStatusPort());
        assertEquals(1200, instance.getRxTrafficEventPort());

        instance.setRxVehicleStatusPort(2100);
        instance.setRxTrafficEventPort(2200);

        assertEquals(2100, instance.getRxVehicleStatusPort());
        assertEquals(2200, instance.getRxTrafficEventPort());

    }

    @Test
    public void testSendVehStatusMsg() throws IOException {
        // Test SendV2xMsg method
        String test_msg = "test message";
        instance.sendVehStatusMsgs(test_msg.getBytes());
        // ArgumentCaptor to capture parameters passed to mock on method calls
        ArgumentCaptor<DatagramPacket> packet = ArgumentCaptor.forClass(DatagramPacket.class);
        // Verify socket.send(DatagramPacket packet) is called and capture packet
        // parameter
        verify(socket, times(1)).send(packet.capture());

        // Verify parameter members
        assertArrayEquals(test_msg.getBytes(), packet.getValue().getData());
        assertEquals(instance.getRxBridgeTimeSyncPort(), packet.getValue().getPort());
    }

    
}
