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
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.fed.infrastructure.ambassador.InfrastructureRegistrationMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoPoint;

/**
 * Session management class for Infrastructure instances communicating with MOSAIC
 * NOTE: TODO See carma.ambassador for reference
 */
public class InfrastructureInstanceManager {
    private Map<String, InfrastructureInstance> managedInstances = new HashMap<>();
    private double currentSimulationTime;

    public void onNewRegistration(InfrastructureRegistrationMessage registration){
        if (!managedInstances.containsKey(registration.getInfrastructureId())) {
            try {
                newInfrastructureInstance(
                    registration.getInfrastructureId(),
                    InetAddress.getByName(registration.getRxMessageIpAddress()),
                    registration.getRxMessagePort(),
                    registration.getTimeSyncPort(),
                    registration.getLocation()
                );
            } catch (UnknownHostException e) {
                throw new RuntimeException(e);
            }
        } else {
            // log warning
        }
    }

    private void newInfrastructureInstance(String infrastructureId, InetAddress rxMessageIpAddress, int rxMessagePort, int timeSyncPort, GeoPoint location) {
        InfrastructureInstance tmp = new InfrastructureInstance(infrastructureId, rxMessageIpAddress, rxMessagePort, timeSyncPort, location);
        try {
            tmp.bind();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        managedInstances.put(infrastructureId, tmp);
    }

        /**
     * External helper function to allow the ambassador to check if a given vehicle ID is a registered CARMA Platform
     * instance
     * @param mosiacVehicleId The id to check
     * @return True if managed by this object (e.g., is a registered CARMA Platform vehicle). false o.w.
     */
    public boolean checkIfRegistered(String infrastructureId) {
        return managedInstances.keySet().contains(infrastructureId);
    }
}
