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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import java.net.InetAddress;
import java.util.ArrayList;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

public class InfrastructureInstanceManagerTest {

    private InfrastructureInstanceManager manager;
    private InfrastructureRegistrationMessage registration;
    private InetAddress ipAddress;
    private CartesianPoint location;

    @Before
    public void setUp() throws Exception {
        manager = new InfrastructureInstanceManager();
        ipAddress = mock(InetAddress.class);
        location = mock(CartesianPoint.class);
    }

    @Test
    public void testOnNewRegistration() {
        // Set up the registration object
        String infrastructureId = "infrastructure-123";
        int rxMessagePort = 1234;
        int timeSyncPort = 5678;
        int simulatedInteractionPort = 2355;
        String ipAddressString = "127.0.0.1";
        CartesianPoint pt = CartesianPoint.xyz(37.3382, -121.8863, 1.0);
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
            new Detector(
                "String sensorId", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 0.0,0.0,0.0),
                CartesianPoint.ORIGO));

        // Mock the behavior of the registration object
        InfrastructureRegistrationMessage registration = new InfrastructureRegistrationMessage(
                                                                            ipAddressString, 
                                                                            infrastructureId, 
                                                                            rxMessagePort, 
                                                                            timeSyncPort, 
                                                                            simulatedInteractionPort ,
                                                                            pt, 
                                                                            sensors);
        // Ensure checkIfRegistered returns false for infrastructure ID before registering 
        assertFalse( manager.checkIfRegistered(infrastructureId) );

        // Call the onNewRegistration method with the mocked registration object
        manager.onNewRegistration(registration);

        // Verify that the infrastructure instance was added to the manager
        assertTrue( manager.checkIfRegistered(infrastructureId) );
        // Ensure checkIfRegistered returns false for other Ids
        assertFalse( manager.checkIfRegistered(infrastructureId + "something") );
    }

}
