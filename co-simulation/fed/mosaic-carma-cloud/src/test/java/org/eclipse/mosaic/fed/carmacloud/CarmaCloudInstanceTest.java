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
package org.eclipse.mosaic.fed.carmacloud.ambassador;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import java.io.BufferedInputStream;
import java.io.DataOutputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.URL;
import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpContext;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import com.google.gson.Gson;
import java.io.IOException;
import java.net.InetAddress;
import org.junit.Before;
import org.junit.Test;



import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaCloudInstanceTest {

    private CarmaCloudInstance instance;

    class TimeSyncHandler implements HttpHandler
	{
        @Override
        public void handle(HttpExchange oExch)
            throws IOException
        {
            int nChar;
            StringBuilder sBuf = new StringBuilder();
            BufferedInputStream oIn = new BufferedInputStream(oExch.getRequestBody());
            while ((nChar = oIn.read()) >= 0)
                sBuf.append((char)nChar);

            System.out.println(sBuf.toString());
            oExch.sendResponseHeaders(200, -1L);
            oExch.close();
        }
    }

    @Before
    public void setup() throws NoSuchFieldException {
        instance = new CarmaCloudInstance("carma-cloud", "http://localhost:8080/carmacloud/simulation");
    }

    @Test
    public void testGetterSetterConstructor() {
        // Test getters and constructor for setting and retrieving class members
        assertEquals("carma-cloud", instance.getCarmaCloudId());
        assertEquals("http://localhost:8080/carmacloud/simulation", instance.getCarmaCloudUrl());
    }

    @Test
    public void testSendTimeSyncMsg() throws IOException {
        // Test SendTimeSyncMsg method
        TimeSyncMessage test_msg = new TimeSyncMessage(100, 1);
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
}
