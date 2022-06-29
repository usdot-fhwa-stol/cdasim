/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */

package org.eclipse.mosaic.fed.carla.carlaconnect;

import java.net.*;
import java.io.*;
import java.util.Arrays;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.eclipse.mosaic.fed.carla.ambassador.CarlaAmbassador;

/**
 * The CarlaConnection used to connect CARLA simulator and CARLA ambassador
 */
public class CarlaConnection implements Runnable {

    // CarlaConnection socket port
    int carlaPort = 8913;

    // CarlaConnection host
    String carlaHostName = "localhost";

    // the data output stream to CARLA simulator
    DataOutputStream toCarlaDataOutputStream;

    // the data input stream from CARLA simulator
    DataInputStream fromCarlaDataInputStream;

    // CarlaConnection server socket
    ServerSocket carlaConnectionServerSocket;

    // CarlaConnection client socket
    Socket carlaSocket = null;

    // the reference to CARLA ambassador
    CarlaAmbassador carlaAmbassador;

    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * This is the customized constructor of CarlaConnection
     *
     * @param carlaHostName This is the CarlaConnection server host name
     *
     * @param carlaPort     This the CarlaConnection client port
     */
    public CarlaConnection(String carlaHostName, int carlaPort, CarlaAmbassador fedAmbassador) {
        this.carlaHostName = carlaHostName;
        this.carlaPort = carlaPort;
        this.carlaAmbassador = fedAmbassador;
    }

    /**
     * This method is used to run the CarlaConnection. After CARLA simulator
     * connected, the message begins to transfer between too ends.
     */
    @Override
    public void run() {
        try {
            // create a carla connection server socket
            carlaConnectionServerSocket = new ServerSocket(carlaPort);
            // Blocking call until CARLA is connected.
            carlaSocket = carlaConnectionServerSocket.accept();
            log.info("Carla Connected");
            try {
                InputStream fromCarlaInputStream = carlaSocket.getInputStream();
                fromCarlaDataInputStream = new DataInputStream(fromCarlaInputStream);
                OutputStream toCarlaOutputStream = carlaSocket.getOutputStream();
                toCarlaDataOutputStream = new DataOutputStream(toCarlaOutputStream);
                log.info("Begin Co-Simulation");
                // Socket data stream ready for transfer message between CARLA simulator and
                // Carla ambassador
                // message buffer
                byte[] buffer = new byte[65535];
                while (true) {
                    // get message from CARLA simulator
                    int length = fromCarlaInputStream.read(buffer);
                    byte[] traciCommandFromCarla = Arrays.copyOfRange(buffer, 0, length);
                    // trigger

                    carlaAmbassador.triggerInteraction(length, traciCommandFromCarla);
                }
            } catch (Exception e) {
                log.error("error occurs during data streaming: " + e.getMessage());
            }
        } catch (Exception e) {
            log.error("error occurs during socket connection: " + e.getMessage());
        }
    }

    /**
     * Get data output stream of socket
     */
    public synchronized DataOutputStream getDataOutputStream() {
        return toCarlaDataOutputStream;
    }

    /**
     * close carla socket and CarlaConnectionServer socket
     */
    public synchronized void closeSocket() {
        try {
            // close carla socket
            log.info("carla socket closing");
            carlaSocket.close();
            // close connection server socket
            log.info("carla connection server socket closing");
            carlaConnectionServerSocket.close();
        } catch (Exception e) {
            log.error("error occurs during closing carla socket: " + e.getMessage());
        }
    }
}
