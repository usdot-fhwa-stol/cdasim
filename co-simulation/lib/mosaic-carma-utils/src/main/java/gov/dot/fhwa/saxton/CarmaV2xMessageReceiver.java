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

package gov.dot.fhwa.saxton;

import org.eclipse.mosaic.lib.misc.Tuple;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * Worker thread Runnable for operating a listen socket to receive outbound V2X Messages from CARMA Platform instances
 * This {@link Runnable} instance will operate a UDP socket to subscribe to packets from the CARMA Platfomr's NS-3
 * adapter. Upon receiving a packet, it will be enqueued for the primary thread to process the data once it ticks to a
 * simulation processing step
 */
public class CarmaV2xMessageReceiver implements Runnable {

    private Queue<Tuple<InetAddress, CarmaV2xMessage>> rxQueue = new LinkedList<>();
    private DatagramSocket listenSocket = null;
    private int listenPort;
    private boolean running = true;
    private static final int UDP_MTU = 1536;

    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Default constructor added to preserve prior behavior after refactor into new package for re-use in Infrastructure
     * Ambassador.
     *
     * Assumes a desired listen port of 1516.
     */
    public CarmaV2xMessageReceiver() {
        this.listenPort = 1516;
    }

    /**
     * Recommended constructor for CarmaV2XMessageReceiver.
     *
     * @param listenPort The host port that should be used to bind a UDP socket for receiving V2X message transmission
     *                   events from connected CARMA software instances. An exception will be thrown in init() if this
     *                   port is unavailable or socket binding fails for other reasons.
     */
    public CarmaV2xMessageReceiver(int listenPort) {
        this.listenPort = listenPort;
    }

    /**
     * Initialize the listen socket for messages from the CARMA Platform NS-3 Adapter
     * @throws RuntimeException iff socket instantiation fails
     */
    public void init() {
        try {
            listenSocket = new DatagramSocket(listenPort);
            log.info("CarmaV2xMessageReceiver started listening on UDP port: " + listenPort);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        byte[] buf = new byte[UDP_MTU];
       while (running) {
           DatagramPacket msg = new DatagramPacket(buf, buf.length);
           InetAddress senderAddr = msg.getAddress();



           try {
               listenSocket.receive(msg);
               log.info("CarmaV2xMessageReceiver received message of size: " + msg.getLength() + " from client " + msg.getAddress().toString() + ".");
           } catch (IOException e) {
               throw new RuntimeException(e);
           }

           // parse message
           try {
               CarmaV2xMessage parsedMessage = new CarmaV2xMessage(msg.getData());

               // Enqueue message for processing on main thread
               synchronized (rxQueue) {
                   rxQueue.add(new Tuple<>(senderAddr, parsedMessage));
                   log.info("CarmaV2xMessageReceiver enqueued message of size: " + msg.getLength() + " from client " + msg.getAddress().toString() + ".");
               }
           } catch (IllegalArgumentException parseError) {
               log.warn("CarmaV2xMessageReceiver received malformed message with length: " + msg.getLength() + " from client " + msg.getAddress().toString() + "! Reason: " + parseError.getMessage() +". Discarding...");
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
    public List<Tuple<InetAddress, CarmaV2xMessage>> getReceivedMessages() {
        List<Tuple<InetAddress, CarmaV2xMessage>> output = new ArrayList<>();

        synchronized (rxQueue) {
            output.addAll(rxQueue);
            rxQueue.clear();
        }
        
        return output;
    }
}
