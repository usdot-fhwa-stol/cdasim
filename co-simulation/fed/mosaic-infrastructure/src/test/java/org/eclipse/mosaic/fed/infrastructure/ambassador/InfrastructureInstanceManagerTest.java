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
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;

import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.mockito.internal.matchers.Any;
import org.mockito.internal.verification.Times;

import com.google.gson.Gson;

public class InfrastructureInstanceManagerTest {

    private InfrastructureInstanceManager manager;
    private InfrastructureInstance instance1;
    private InfrastructureInstance instance2;
    private InfrastructureInstance instance3;

    @Before
    public void setUp() throws Exception {
        manager = new InfrastructureInstanceManager();
        instance1 = mock(InfrastructureInstance.class);
        instance2 = mock(InfrastructureInstance.class);
        instance3 = mock(InfrastructureInstance.class);
        // Add mocks to managed instances
        manager.getManagedInstances().put("instance1", instance1);
        manager.getManagedInstances().put("instance2", instance2);
        manager.getManagedInstances().put("instance3", instance3);
        
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

    @Test
    public void testOnTimeStepUpdate() throws IOException {
        InfrastructureTimeMessage message = new InfrastructureTimeMessage();
    
        message.setSeq(3);
        message.setTimestep(300);
        manager.onTimeStepUpdate(message);
        Gson gson = new Gson();
        byte[] datagram = gson.toJson(message).getBytes();
        // Verify that all instances sendTimeSyncMsgs was called.
        verify(instance1).sendTimeSyncMsgs(datagram);
        verify(instance2).sendTimeSyncMsgs(datagram);
        verify(instance2).sendTimeSyncMsgs(datagram);

    }

    @Test
    public void testOnDetectedObject() throws IOException{
        when(instance1.containsSensor("sensor1")).thenReturn(true);
        when(instance1.containsSensor("sensor2")).thenReturn(true);
        when(instance2.containsSensor("sensor3")).thenReturn(true);
        when(instance2.containsSensor("sensor4")).thenReturn(true);
        when(instance3.containsSensor("sensor5")).thenReturn(true);
        when(instance3.containsSensor("sensor6")).thenReturn(true);
        DetectedObject detectedObject = new DetectedObject(
                DetectionType.VAN,
                0.5,
                "sensor1",
                "projection String",
                "Object1",
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(2, 3, 4),
                new Vector3d(-4.4,-5.5,-6.6),
                new Size(3, 4, 5));
        Double[] covarianceMatrix = new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);

        manager.onObjectDetectionInteraction(detectedObject);
        Gson gson = new Gson();
        byte[] datagram = gson.toJson(detectedObject).getBytes();
        verify(instance1, times(1)).sendInteraction(datagram);


    }

}
