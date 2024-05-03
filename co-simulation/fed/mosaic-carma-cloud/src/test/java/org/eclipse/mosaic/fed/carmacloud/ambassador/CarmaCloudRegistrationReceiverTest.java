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

 import static org.junit.Assert.assertEquals;
 import static org.mockito.Mockito.mock;
 import static org.mockito.Mockito.when;
 import org.mockito.internal.util.reflection.FieldSetter;

 import java.io.ByteArrayInputStream;
 import java.io.ByteArrayOutputStream;
 import java.io.DataOutputStream;
 import java.net.InetAddress;
 import java.net.InetSocketAddress;
 import java.net.ServerSocket;
 import java.net.Socket;
 import java.util.List;
 
 import org.junit.After;
 import org.junit.Before;
 import org.junit.Test;
 
 public class CarmaCloudRegistrationReceiverTest {
  
     @Test
     public void testMessageReceive() throws Exception {
         // Define a test message in JSON format
         String json = "{\"id\":\"carma-cloud\",\"url\":\"http://someaddress:8080/carmacloud/simulation\"}";
         ByteArrayOutputStream oBytes = new ByteArrayOutputStream();
         DataOutputStream oOut = new DataOutputStream(oBytes);
         oOut.writeUTF(json);
         oOut.close(); // flush contents
      
         ServerSocket MockServer = mock(ServerSocket.class);
         Socket MockSock = mock(Socket.class);

         // mock socket server, socket, and inputstream
         when(MockServer.accept()).thenReturn(MockSock);
         when(MockSock.getInputStream()).thenReturn(new ByteArrayInputStream(oBytes.toByteArray()));

         // Setup the registration receiver
         CarmaCloudRegistrationReceiver receiver = new CarmaCloudRegistrationReceiver();
         FieldSetter.setField(receiver, receiver.getClass().getDeclaredField("srvr"), MockServer);
         receiver.run(); // multi-threading not needed here

         // Verify that the message was received correctly
         List<CarmaCloudRegistrationMessage> msgs = receiver.getReceivedMessages();
         assertEquals(1, msgs.size());
 
         CarmaCloudRegistrationMessage msg = msgs.get(0);
         assertEquals("carma-cloud", msg.getId());
         assertEquals("http://someaddress:8080/carmacloud/simulation", msg.getUrl());
     }
 }
 
