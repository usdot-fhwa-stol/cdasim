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
import java.util.LinkedList;
import java.util.Queue;

import org.eclipse.mosaic.lib.CommonUtil.CommonRegistrationReceiver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

public class CarmaMessengerRegistrationReceiver extends CommonRegistrationReceiver<CarmaMessengerRegistrationMessage>{
    
    public CarmaMessengerRegistrationReceiver(Class<CarmaMessengerRegistrationMessage> type) {
        super(type);
        //TODO Auto-generated constructor stub
    }

    private Queue<CarmaMessengerRegistrationMessage> rxQueue = new LinkedList<>();
    private DatagramSocket listenSocket = null;
    private static final int listenPort = 1715;
    private boolean running = true;
    private static final int UDP_MTU = 1535;
    private final Logger log = LoggerFactory.getLogger(this.getClass());



    @Override
    public void run() {
        byte[] buf = new byte[UDP_MTU];
        while (running) {
            DatagramPacket msg = new DatagramPacket(buf, buf.length);
            try {
               listenSocket.receive(msg);
            } catch (IOException e) {
               throw new RuntimeException(e);
            }

            // parse message
            String receivedPacket = new String(msg.getData(), 0, msg.getLength());
            log.debug("Registration JSON received:  {}", receivedPacket);
            Gson gson = new Gson();
            CarmaMessengerRegistrationMessage parsedMessage = gson.fromJson(receivedPacket, CarmaMessengerRegistrationMessage.class);

            // Enqueue message for processing on main thread
            synchronized (rxQueue) {
                log.info("New CARMA messenger instance '{}' received with CARMA messenger Registration Receiver.", parsedMessage.getVehicleId());
                rxQueue.add(parsedMessage);
            }
       }
    }

}
