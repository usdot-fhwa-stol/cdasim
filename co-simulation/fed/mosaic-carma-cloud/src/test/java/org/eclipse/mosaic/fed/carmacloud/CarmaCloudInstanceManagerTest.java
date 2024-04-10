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
 
package org.eclipse.mosaic.fed.carmacloud.ambassador;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.TimeSyncMessage;



public class CarmaCloudInstanceManagerTest {

    private CarmaCloudInstanceManager manager;
    private CarmaCloudInstance instance1;
    private CarmaCloudInstance instance2;
    private CarmaCloudInstance instance3;

    @Before
    public void setUp() throws Exception {
        manager = new CarmaCloudInstanceManager();
        instance1 = mock(CarmaCloudInstance.class);
        instance2 = mock(CarmaCloudInstance.class);
        instance3 = mock(CarmaCloudInstance.class);
        // Add mocks to managed instances
        manager.getManagedInstances().put("instance1", instance1);
        manager.getManagedInstances().put("instance2", instance2);
        manager.getManagedInstances().put("instance3", instance3);
        // Mocks will account for detected objects with the given sensor IDs.
        when(instance1.containsSensor("sensor1")).thenReturn(true);
        when(instance1.containsSensor("sensor2")).thenReturn(true);
        when(instance2.containsSensor("sensor3")).thenReturn(true);
        when(instance2.containsSensor("sensor4")).thenReturn(true);
        when(instance3.containsSensor("sensor5")).thenReturn(true);
        when(instance3.containsSensor("sensor6")).thenReturn(true);
        
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
        CarmaCloudRegistrationMessage registration = new CarmaCloudRegistrationMessage(
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
        TimeSyncMessage message = new TimeSyncMessage(300, 3);
        manager.onTimeStepUpdate(message);
        // Verify that all instances sendTimeSyncMsgs was called.
        verify(instance1).sendTimeSyncMsg(message);
        verify(instance2).sendTimeSyncMsg(message);
        verify(instance2).sendTimeSyncMsg(message);

    }

    @Test
    public void testOnDetectedObject() throws IOException{
        // Create detected object
        DetectedObject detectedObject1 = new DetectedObject(
                DetectionType.VAN,
                0.5,
                "sensor1",
                "projection String",
                100,
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(2, 3, 4),
                new Vector3d(-4.4,-5.5,-6.6),
                new Size(3, 4, 5),
                100);
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        detectedObject1.setPositionCovariance(covarianceMatrix);
        detectedObject1.setVelocityCovariance(covarianceMatrix);
        detectedObject1.setAngularVelocityCovariance(covarianceMatrix);
        // Attempt to send detected object to infrastructure instance
        manager.onDetectedObject(detectedObject1);
        // Verify CarmaCloud Manager attempted to sent Detected Object
        // to instance1
        verify(instance1, times(1)).sendDetection(detectedObject1);
        // Create second detected object
        DetectedObject detectedObject2 = new DetectedObject(
                DetectionType.VAN,
                0.5,
                "sensor6",
                "projection String",
                101,
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(2, 3, 4),
                new Vector3d(-4.4,-5.5,-6.6),
                new Size(3, 4, 5),
                100);
        detectedObject2.setPositionCovariance(covarianceMatrix);
        detectedObject2.setVelocityCovariance(covarianceMatrix);
        detectedObject2.setAngularVelocityCovariance(covarianceMatrix);
        manager.onDetectedObject(detectedObject2);
        doThrow(new IOException("Something went wrong")).when(instance3).sendDetection(detectedObject2);
        verify(instance3, times(1)).sendDetection(detectedObject2);
        verify(instance2, never()).sendDetection(any(DetectedObject.class));
    }

}
