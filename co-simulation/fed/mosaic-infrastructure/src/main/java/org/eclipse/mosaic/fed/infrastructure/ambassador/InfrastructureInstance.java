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
import java.net.InetAddress;
import java.util.List;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.rti.api.Interaction;

import com.google.gson.Gson;

/**
 * InfrastructureInstance class represents a physical instance of an
 * infrastructure instance in the simulated environment.
 * It contains information about the infrastructure instance such as its ID,
 * location, target address, and ports.
 */
public class InfrastructureInstance {

    private String infrastructureId;
    private InetAddress targetAddress;
    private int rxMessagePort;
    private int timeSyncPort;
    private int simulatedInteractionPort;
    private CartesianPoint location = null;
    private DatagramSocket socket = null;
    private List<Detector> sensors;

    /**
     * Constructor for InfrastructureInstance
     * 
     * @param infrastructureId          the ID of the infrastructure instance.
     * @param targetAddress             the target IP address of the infrastructure instance.
     * @param rxMessagePort             the receive V2X message port of the infrastructure instance.
     * @param timeSyncPort              the receive time synchronization message port of the infrastructure.
     *                         
     * @param simulatedInteractionPort  the receive simulated interaction message port of the infrastructure
     *                                  instance.
     * @param location                  the location of the infrastructure instance in the
     *                                  simulated environment
     */ 
    public InfrastructureInstance(String infrastructureId, InetAddress targetAddress, int rxMessagePort,
            int timeSyncPort, int simulatedInteractionPort, CartesianPoint location, List<Detector> sensors) {
        this.infrastructureId = infrastructureId;
        this.targetAddress = targetAddress;
        this.rxMessagePort = rxMessagePort;
        this.timeSyncPort = timeSyncPort;
        this.simulatedInteractionPort = simulatedInteractionPort;
        this.location = location;
        this.sensors = sensors;
    }



    /**
     * Returns the target IP address of the infrastructure instance
     * 
     * @return InetAddress the target IP address of the infrastructure instance
     */
    public InetAddress getTargetAddress() {
        return targetAddress;
    }

    /**
     * Sets the target IP address of the infrastructure instance
     * 
     * @param targetAddress the target IP address to set
     */
    public void setTargetAddress(InetAddress targetAddress) {
        this.targetAddress = targetAddress;
    }

    /**
     * Returns the location of the infrastructure instance in the simulated environment
     * 
     * @return CartesianPoint the location of the infrastructure instance
     */
    public CartesianPoint getLocation() {
        return this.location;
    }

    /**
     * Sets the location of the infrastructure instance in the simulated environment
     * 
     * @param location the location to set
     */
    public void setLocation(CartesianPoint location) {
        this.location = location;
    }

    /**
     * Returns the ID of the infrastructure instance
     * 
     * @return String the ID of the infrastructure instance
     */
    public String getInfrastructureId() {
        return infrastructureId;
    }

    /**
     * Sets the ID of the infrastructure instance
     * 
     * @param infrastructureId the ID to set
     */
    public void setInfrastructureId(String infrastructureId) {
        this.infrastructureId = infrastructureId;
    }

    /**
     * Returns the receive message port of the infrastructure instance
     * 
     * @return int the receive message port of the infrastructure instance
     */
    public int getRxMessagePort() {
        return rxMessagePort;
    }

    /**
     * Sets the receive message port of the infrastructure instance
     * 
     * @param rxMessagePort the port to set
     */
    public void setRxMessagePort(int rxMessagePort) {
        this.rxMessagePort = rxMessagePort;
    }

    /**
     * Returns the time synchronization port of the infrastructure instance
     * 
     * @return int the time synchronization port of the infrastructure instance
     */
    public int getTimeSyncPort() {
        return timeSyncPort;
    }

    /**
     * Sets the time synchronization port of the infrastructure instance
     * 
     * @param timeSyncPort the port to set
     */
    public void setTimeSyncPort(int timeSyncPort) {
        this.timeSyncPort = timeSyncPort;
    }
    
    /**
     * Returns the simulated interaction port of the infrastructure instance.
     * 
     * @return int simulated interaction port of the infrastructure instance.
     */
    public int getSimulatedInteractionPort() {
        return simulatedInteractionPort;
    }


    /**
     * Sets the simulated interaction port of the infrastructure instance.
     * 
     * @param simulatedInteractionPort int simulated interaction port of the infrastructure
     * instance.
     */
    public void setSimulatedInteractionPort(int simulatedInteractionPort) {
        this.simulatedInteractionPort = simulatedInteractionPort;
    }


    /**
     * Returns list of Sensor/Detectors registered to this infrastructure instance.
     * 
     * @return list of detectors registered to infrastructure instance.
     */
    public List<Detector> getSensors() {
        return sensors;
    }


    /**
     * Sets list of Sensors/Detectors registered to infrastructure instance.
     * 
     * @param sensors list of detectors registered to infrastructure instance.
     */
    public void setSensors(List<Detector> sensors) {
        this.sensors = sensors;
    }


    /**
     * Method that returns boolean value based on whether any registered Sensor/Detector
     * in the sensor list has a sensorId equivalent to the passed parameter.
     * 
     * @param sensorId sensor ID to search for 
     * @return true if sensor with sensor ID exists in sensor list. False otherwise.
     */
    public boolean containsSensor(String sensorId) {
        for (Detector sensor : sensors) {
            if (sensor.getSensorId().equals(sensorId) ) {
                return true;
            } 
        }
        return false;
    }

    /**
     * Creates a DatagramSocket object
     * 
     * @throws IOException if there is an issue with the underlying socket object
     */
    public void connect() throws IOException {
        socket = new DatagramSocket();
    }

    /**
     * Sends the data to the Infrastructure Device communications interface
     * configured at construction time.
     * 
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or
     *                     methods
     */
    public void sendV2xMsg(byte[] data) throws IOException {
        sendPacket(data, rxMessagePort);
    }

    /**
     * Sends time sync data to the Infrastrucutre Instance Time Sync port.
     * 
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendTimeSyncMsg(InfrastructureTimeMessage message) throws IOException {
        sendPacket(toJsonBytes(message), timeSyncPort);
    }

    /**
     * Helper method to serialize message into JSON and encode as bytes.
     * 
     * @param message java object containing message information
     * @return bytes encoded from JSON string representation of object.
     */
    private byte[] toJsonBytes(Object message) {
        Gson gson = new Gson();
        return gson.toJson(message).getBytes();
    }

    /**
     * Sends time sync data to the Infrastrucutre Instance Simulated Interaction port.
     * 
     * @param data The binary data to transmit
     * @throws IOException If there is an issue with the underlying socket object or methods
     */
    public void sendDetection(DetectedObject detectedObject) throws IOException {
        sendPacket(toJsonBytes(detectedObject), simulatedInteractionPort);
    }
    /**
     * Method to send byte data to specified port using the infrastructure instance Datagramsocket.
     * 
     * @param data byte data to send using Datagramsocket.
     * @param port in integer format to send Datagram to.
     * @throws IOException
     */
    private void sendPacket(byte[] data, int port) throws IOException {
         if (socket == null) {
            throw new IllegalStateException("Attempted to send data before opening socket");
        }
        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, port);
        socket.send(packet);
    }
}
