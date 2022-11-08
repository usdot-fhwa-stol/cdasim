/*
 * Copyright (C) 2019-2022 LEIDOS.
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


package org.eclipse.mosaic.fed.carma.ambassador;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/**
 * Worker thread Runnable for operating a listen socket to receive outbound V2X Messages from CARMA Platform instances
 * This {@link Runnable} instance will operate a UDP socket to subscribe to packets from the CARMA Platfomr's NS-3
 * adapter. Upon receiving a packet, it will be enqueued for the primary thread to process the data once it ticks to a
 * simulation processing step
 */
public class CarmaV2xMessageReceiver implements Runnable {

    private Queue<CarmaV2xMessage> rxQueue = new ArrayList<>();
    private DatagramSocket listenSocket = null;
    private static final int listenPort = 1516;
    private boolean running = true;
    private static final int UDP_MTU = 1535;

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
            CarmaV2xMessage parsedMessage = new CarmaV2xMessage(msg.getData());

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
     * @return The list of received outbound message from all CARMA Platform instances since last call of this method
     */
    public List<CarmaV2xMessage> getReceivedMessages() {
        List<CarmaV2xMessage> output = new ArrayList<>();

        synchronized (rxQueue) {
            output.addAll(rxQueue);
            rxQueue.clear();
        }
        
        return output;
    }
}
