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

import java.util.List;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;

/**
 * A message to be sent by Infrastructure instance when it registers with the
 * carma-mosaic ambassador
 */
public class InfrastructureRegistrationMessage {

    // IP address where the Infrastructure instance will be listening for inbound
    // messages
    private String rxMessageIpAddress;
    // Unique identifier for the Infrastructure instance
    private String infrastructureId;

    // Port number where the Infrastructure instance will be listening for inbound
    // messages
    private int rxMessagePort;

    // Port number where the Infrastructure instance will be listening for time
    // synchronization messages
    private int timeSyncPort;
    
    // Port number where infrastructure instance will be listening for interaction 
    // messages
    private int simulatedInteractionPort;

    // Geo-coordinate of the Infrastructure instance location
    private CartesianPoint location = null;

    // List of Sensor/Detectors associated with infrastructure instance.
    private List<Detector> sensors;


    /**
     * Constructor for an `InfrastructureRegistrationMessage` instance
     * 
     * @param rxMessageIpAddress        IP address where the Infrastructure instance will be
     *                                  listening for inbound messages
     * @param infrastructureId          Unique identifier for the Infrastructure instance
     * @param rxMessagePort             Port number where the Infrastructure instance will be
     *                                  listening for inbound messages
     * @param simulatedInteractionPort  Port number where Infrastructure instance will be listening
     *                                  for simulated interactions.
     * @param timeSyncPort              Port number where the Infrastructure instance will be
     *                                  listening for time synchronization messages
     * @param location                  Geo-coordinate of the Infrastructure instance
     *                                  location
     * @param sensors                   ArrayList of sensors to register for a given infrastructure
     *                                  instance
     */
    public InfrastructureRegistrationMessage(String rxMessageIpAddress, String infrastructureId, int rxMessagePort,
            int timeSyncPort, int simulatedInteractionPort, CartesianPoint location, List<Detector> sensors) {
        this.rxMessageIpAddress = rxMessageIpAddress;
        this.infrastructureId = infrastructureId;
        this.rxMessagePort = rxMessagePort;
        this.timeSyncPort = timeSyncPort;
        this.simulatedInteractionPort = simulatedInteractionPort;
        this.location = location;
        this.sensors = sensors;
    }



    /**
     * Returns the IP address where the Infrastructure instance will be listening for
     * inbound messages
     * 
     * @return The IP address where the Infrastructure instance will be listening for
     *         inbound messages
     */
    public String getRxMessageIpAddress() {
        return this.rxMessageIpAddress;
    }

    /**
     * Returns the unique identifier for the Infrastructure instance
     * 
     * @return The unique identifier for the Infrastructure instance
     */
    public String getInfrastructureId() {
        return this.infrastructureId;
    }

    /**
     * Returns the port number where the Infrastructure instance will be listening for
     * inbound messages
     * 
     * @return The port number where the Infrastructure instance will be listening for
     *         inbound messages
     */
    public int getRxMessagePort() {
        return this.rxMessagePort;
    }

    /**
     * Returns the port number where the Infrastructure instance will be listening for
     * time synchronization messages
     * 
     * @return The port number where the Infrastructure instance will be listening for
     *         time synchronization messages
     */
    public int getTimeSyncPort() {
        return this.timeSyncPort;
    }

    /**
     * Returns the Geo-coordinate of the Infrastructure instance location
     * 
     * @return The Geo-coordinate of the Infrastructure instance location
     */
    public CartesianPoint getLocation() {
        return this.location;
    }

    /**
     * Returns list of associated sensors/detectors.
     * 
     * @return list of sensors/detectors registered to infrastructure instances.
     */
    public List<Detector> getSensors() {
        return sensors;
    }
    
    /**
     * Returns port where infrastructure instance will be listening for interaction messages
     * from CDASim.
     * 
     * @return int simulated interaction port
     */
    public int getSimulatedInteractionPort() {
        return simulatedInteractionPort;
    }

    @Override
    public String toString() {
        return "InfrastructureRegistrationMessage [rxMessageIpAddress=" + rxMessageIpAddress + ", infrastructureId="
                + infrastructureId + ", rxMessagePort=" + rxMessagePort + ", timeSyncPort=" + timeSyncPort
                + ", simulatedInteractionPort=" + simulatedInteractionPort + ", location=" + location + ", sensors="
                + sensors + "]";
    }

}
