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

import org.eclipse.mosaic.lib.geo.CartesianPoint;

/**
 * A message to be sent by Infrastructure Device when it registers with the
 * carma-mosaic ambassador
 */
public class InfrastructureRegistrationMessage {

    // IP address where the Infrastructure Device will be listening for inbound
    // messages
    private String rxMessageIpAddress;
    // Unique identifier for the Infrastructure Device
    private String infrastructureId;

    // Port number where the Infrastructure Device will be listening for inbound
    // messages
    private int rxMessagePort = 1536;

    // Port number where the Infrastructure Device will be listening for time
    // synchronization messages
    private int timeSyncPort = 1517;

    // Geo-coordinate of the Infrastructure Device location
    private CartesianPoint location = null;

    /**
     * Constructor for an `InfrastructureRegistrationMessage` instance
     * 
     * @param rxMessageIpAddress IP address where the Infrastructure Device will be
     *                           listening for inbound messages
     * @param infrastructureId   Unique identifier for the Infrastructure Device
     * @param rxMessagePort      Port number where the Infrastructure Device will be
     *                           listening for inbound messages
     * @param timeSyncPort       Port number where the Infrastructure Device will be
     *                           listening for time synchronization messages
     * @param location           Geo-coordinate of the Infrastructure Device
     *                           location
     */
    public InfrastructureRegistrationMessage(String rxMessageIpAddress, String infrastructureId,
            int rxMessagePort, int timeSyncPort, CartesianPoint location) {
        this.rxMessageIpAddress = rxMessageIpAddress;
        this.infrastructureId = infrastructureId;
        this.rxMessagePort = rxMessagePort;
        this.timeSyncPort = timeSyncPort;
        this.location = location;
    }

    /**
     * Returns the IP address where the Infrastructure Device will be listening for
     * inbound messages
     * 
     * @return The IP address where the Infrastructure Device will be listening for
     *         inbound messages
     */
    public String getRxMessageIpAddress() {
        return this.rxMessageIpAddress;
    }

    /**
     * Returns the unique identifier for the Infrastructure Device
     * 
     * @return The unique identifier for the Infrastructure Device
     */
    public String getInfrastructureId() {
        return this.infrastructureId;
    }

    /**
     * Returns the port number where the Infrastructure Device will be listening for
     * inbound messages
     * 
     * @return The port number where the Infrastructure Device will be listening for
     *         inbound messages
     */
    public int getRxMessagePort() {
        return this.rxMessagePort;
    }

    /**
     * Returns the port number where the Infrastructure Device will be listening for
     * time synchronization messages
     * 
     * @return The port number where the Infrastructure Device will be listening for
     *         time synchronization messages
     */
    public int getTimeSyncPort() {
        return this.timeSyncPort;
    }

    /**
     * Returns the Geo-coordinate of the Infrastructure Device location
     * 
     * @return The Geo-coordinate of the Infrastructure Device location
     */
    public CartesianPoint getLocation() {
        return this.location;
    }

    /**
     * Sets the IP address where the Infrastructure Device will be listening for
     * inbound messages
     * 
     * @param rxMessageIpAddress The IP address where the Infrastructure Device will
     *                           be listening for inbound messages
     */
    public void setRxMessageIpAddress(String rxMessageIpAddress) {
        this.rxMessageIpAddress = rxMessageIpAddress;
    }

    /**
     * Sets the unique identifier for the Infrastructure Device
     * It currently is the same with RSU ID
     * 
     * @param infrastructureId The unique identifier for the Infrastructure Device
     */
    public void setInfrastructureId(String infrastructureId) {
        this.infrastructureId = infrastructureId;
    }

    /**
     * Sets the port number where the Infrastructure Device will be listening for
     * inbound messages
     * 
     * @param rxMessagePort The port number where the Infrastructure Device will be
     *                      listening for inbound messages
     */
    public void setRxMessagePort(int rxMessagePort) {
        this.rxMessagePort = rxMessagePort;
    }

    /**
     * Set the time sync port for the InfrastructureRegistrationMessage
     * 
     * @param timeSyncPort the new time sync port to be set
     */
    public void setTimeSyncPort(int timeSyncPort) {
        this.timeSyncPort = timeSyncPort;
    }

    /**
     * Set the location for the InfrastructureRegistrationMessage
     * 
     * @param location the new GeoPoint object representing the location of the
     *                 Infrastructure Device
     */
    public void setLocation(CartesianPoint location) {
        this.location = location;
    }

    /**
     * Returns a string representation of the InfrastructureRegistrationMessage
     * object
     * 
     * @return a string representation of the object
     */
    @Override
    public String toString() {
        return "InfrastructureRegistrationMessage [rxMessageIpAddress=" + rxMessageIpAddress
                + ", infrastructureId=" + infrastructureId + ", rxMessagePort=" + rxMessagePort
                + ", timeSyncPort=" + timeSyncPort + ", location=" + location + "]";
    }

}
