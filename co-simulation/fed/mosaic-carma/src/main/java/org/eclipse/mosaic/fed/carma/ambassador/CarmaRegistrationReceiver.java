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

import com.google.gson.Gson;

import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonRegistrationReceiver;

/**
 * Worker thread Runnable for operating a listen socket to receive outbound V2X Messages from CARMA Platform instances
 * This {@link Runnable} instance will operate a UDP socket to subscribe to packets from the CARMA Platfomr's NS-3
 * adapter. Upon receiving a packet, it will be enqueued for the primary thread to process the data once it ticks to a
 * simulation processing step
 */
public class CarmaRegistrationReceiver extends CommonRegistrationReceiver<CarmaRegistrationMessage>{
    
    public CarmaRegistrationReceiver(Class<CarmaRegistrationMessage> type) {
        super(type);
    }

    private DatagramSocket listenSocket = null;
    private static final int listenPort = 1515;
    private boolean running = true;
    private static final int UDP_MTU = 1535;


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
            CarmaRegistrationMessage parsedMessage = gson.fromJson(receivedPacket, CarmaRegistrationMessage.class);

            // Enqueue message for processing on main thread
            synchronized (rxQueue) {
                log.info("New CARMA instance '{}' received with CARMA Registration Receiver.", parsedMessage.getVehicleId());
                rxQueue.add(parsedMessage);
            }
       }
    }
}
