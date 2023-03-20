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

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import com.google.gson.Gson;

/**
 * Worker thread Runnable for operating a listen socket to receive outbound V2X Messages from Infrastructure Device instances
 * This {@link Runnable} instance will operate a UDP socket to subscribe to packets from the V2x infrastructure's
 * adapter. Upon receiving a packet, it will be enqueued for the primary thread to process the data once it ticks to a
 * simulation processing step
 * NOTE: TODO See carma.ambassador for reference
 */
public class V2xRegistrationReceiver implements Runnable {
    private Queue<V2xRegistrationMessage> rxQueue = new LinkedList<>();
    private DatagramSocket listenSocket = null;
    private static final int listenPort = 1515; // TODO
    private boolean running = false;
    private static final int UDP_MTU = 1535;    // TODO Maximum Transmission Unit

    /**
     * Initialize the listen socket for messages from the Infrastructure Device NS-3 Adapter
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
        running = true;
        while (running) {
            DatagramPacket msg = new DatagramPacket(buf, buf.length);
            try {
               listenSocket.receive(msg);
            } catch (IOException e) {
               throw new RuntimeException(e);
            }

            // parse message
            String receivedPacket = new String(msg.getData());
            Gson gson = new Gson();
            V2xRegistrationMessage parsedMessage = gson.fromJson(receivedPacket, V2xRegistrationMessage.class);

            // Enqueue message for processing on main thread
            synchronized (rxQueue) {
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
     * @return The list of received outbound message from all Infrastructure Device instances since last call of this method
     */
    public List<V2xRegistrationMessage> getReceivedMessages() {
        List<V2xRegistrationMessage> output = new ArrayList<>();

        synchronized (rxQueue) {
            output.addAll(rxQueue);
            rxQueue.clear();
        }
        
        return output;
    }
}
