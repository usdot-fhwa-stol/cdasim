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
import java.net.SocketException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import com.google.gson.Gson;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class CarmaMessengerRegistrationReceiver implements Runnable{
    private Queue<CarmaMessengerRegistrationMessage> rxQueue = new LinkedList<>();
    private DatagramSocket listenSocket = null;
    private static final int listenPort = 1515;
    private boolean running = true;
    private static final int UDP_MTU = 1535;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Initialize the listen socket for messages from the CARMA Platform NS-3 Adapter
     * @throws RuntimeException iff socket instantiation fails
     */
    public void init() {
        try {
            listenSocket = new DatagramSocket(listenPort);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
    }

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
                log.info("New CARMA messenger instance '{}' received with CARMA messenger Registration Receiver.", parsedMessage.getCarmaVehicleId());
                rxQueue.add(parsedMessage);
            }
       }
    }

    /**
     * Stop the runnable instance
     */
    public void stop() {
        if (listenSocket != null) {
            listenSocket.close();
        }

        running = false;
    }

    /**
     * Query the current buffer of outbound messages. Clears the currently stored buffer once called. Thread-safe.
     * @return The list of received outbound message from all CARMA Platform instances since last call of this method
     */
    public List<CarmaMessengerRegistrationMessage> getReceivedMessages() {
        List<CarmaMessengerRegistrationMessage> output = new ArrayList<>();

        synchronized (rxQueue) {
            output.addAll(rxQueue);
            rxQueue.clear();
        }
        
        return output;
    }
}
