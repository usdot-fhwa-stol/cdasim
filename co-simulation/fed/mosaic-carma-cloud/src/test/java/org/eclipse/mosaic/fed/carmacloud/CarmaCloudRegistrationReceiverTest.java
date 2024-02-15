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
 
 import java.net.Socket;
 import java.net.InetAddress;
 import java.util.List;
 
 import org.junit.After;
 import org.junit.Before;
 import org.junit.Test;
 
 public class CarmaCloudRegistrationReceiverTest {
 
     private static final int TEST_PORT = 1617;
 
     private Socket sendSocket;
     private CarmaCloudRegistrationReceiver receiver;
 
     @Before
     public void setup() throws Exception {
         // Set up a TCP socket to send messages
         sendSocket = new Socket();
 
         // Initialize the receiver
         receiver = new CarmaCloudRegistrationReceiver();
         receiver.init();
         Thread receiverThread = new Thread(receiver);
         receiverThread.start();
     }
 
     @After
     public void teardown() throws Exception {
         // Stop the receiver and close the send socket
         receiver.stop();
         sendSocket.close();
     }
 
     @Test
     public void testMessageReceive() throws Exception {
         // Define a test message in JSON format
         String json = "{\"id\":\"carma-cloud\",\"url\":\"http://someaddress:8080/carmacloud/simulation\"}";
 
         // Send the test message to the receiver
         InetAddress address = InetAddress.getLocalHost();
         sendSocket.connect(new InetSocketAddress(address, 1617), 10000);
				     DataOutputStream out = new DataOutputStream(sendSocket.getOutputStream());
				     out.writeUTF(String.format("{\"id\":\"carma-cloud\", \"url\":\"%s\"}", m_sCarmaCloudUrl));
         out.close();
 
         // Wait for the message to be received
         Thread.sleep(1000);
 
         // Verify that the message was received correctly
         List<CarmaCloudRegistrationMessage> msgs = receiver.getReceivedMessages();
         assertEquals(1, msgs.size());
 
         CarmaCloudRegistrationMessage msg = msgs.get(0);
         assertEquals("carma-cloud", msg.getId());
         assertEquals("http://someaddress:8080/carmacloud/simulation", msg.getUrl());
     }
 }
 
